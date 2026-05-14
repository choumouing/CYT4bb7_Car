#include "menu_air_support.h"

#define MENU_AIR_SLOT_BASE_PAGE             (80U)
#define MENU_AIR_SLOT_COUNT                 (4U)
#define MENU_AIR_SLOT_SIZE                  (2U)
#define MENU_AIR_MAGIC_NUMBER               (0x41495250UL)
#define MENU_AIR_VERSION                    (2U)
#define MENU_AIR_MAX_PARAMS                 (100U)
#define MENU_AIR_SYNC_INVALID_INDEX         (0xFFU)
#define MENU_AIR_ACK_TYPE_SET_PARAM         (0x01U)
#define MENU_AIR_ACK_TYPE_COMMAND           (0x03U)
#define MENU_AIR_COMMIT_WAIT_MS             (2000U)
#define MENU_AIR_COMMAND_TIMEOUT_MS         (1000U)

typedef struct
{
    const char *name;
    uint8 mode;
} menu_air_command_config_t;

typedef struct
{
    uint32 magic;
    uint8 version;
    uint8 slot_id;
    uint16 param_count;
    uint32 checksum;
} menu_air_slot_header_t;

static float s_air_param_values[MENU_AIR_MAX_PARAMS];
static menu_air_param_config_t s_air_params[MENU_AIR_MAX_PARAMS];
static uint8 s_air_param_count;
static uint8 s_air_param_dirty[MENU_AIR_MAX_PARAMS];
static menu_air_sync_status_t s_air_sync_status;
static uint8 s_air_sync_next_index;
static uint8 s_air_last_online;
static uint8 s_air_boot_sync_done;
static uint8 s_air_pending_sync_reason;
static menu_air_cmd_status_t s_air_cmd_status;
static menu_air_command_config_t s_air_commands[MENU_AIR_COMMAND_TABLE_MAX];
static uint8 s_air_command_count;
static uint8 menu_load_air_slot_internal(uint8 slot, uint8 require_online);

static uint8 menu_air_register_command_internal(const char *name, uint8 mode)
{
    uint8 index;
    uint16 name_len;

    if((name == NULL) ||
       (mode > MENU_AIR_COMMAND_MODE_INSTANT))
    {
        return 1U;
    }

    name_len = (uint16)strlen(name);
    if((name_len == 0U) || (name_len > AIR_COMM_COMMAND_NAME_MAX))
    {
        return 1U;
    }

    for(index = 0U; index < s_air_command_count; index++)
    {
        if(strcmp(s_air_commands[index].name, name) == 0)
        {
            s_air_commands[index].mode = mode;
            return 0U;
        }
    }

    if(s_air_command_count >= MENU_AIR_COMMAND_TABLE_MAX)
    {
        return 1U;
    }

    s_air_commands[s_air_command_count].name = name;
    s_air_commands[s_air_command_count].mode = mode;
    s_air_command_count++;

    return 0U;
}

static void menu_air_register_default_commands(void)
{
    (void)menu_air_register_polling_command("show_imu_data");
    (void)menu_air_register_polling_command("show_optical_flow_data");
    (void)menu_air_register_instant_command("beep");
}

static uint8 menu_air_dirty_count(void)
{
    uint8 index;
    uint8 count = 0U;

    for(index = 0U; index < s_air_param_count; index++)
    {
        if(s_air_param_dirty[index] != 0U)
        {
            count++;
        }
    }

    return count;
}

static uint8 menu_air_ack_is_success(uint8 result, uint8 status)
{
    return ((((result == AIR_COMM_ACK_RESULT_OK) && (status == AIR_COMM_STATUS_OK)) ||
             ((result == AIR_COMM_ACK_RESULT_ERROR) && (status == AIR_COMM_STATUS_OUT_OF_RANGE))) ? 1U : 0U);
}

static uint8 menu_air_cmd_ack_text_is(const char *text, const char *prefix)
{
    if((text == NULL) || (prefix == NULL))
    {
        return 0U;
    }

    return (strncmp(text, prefix, strlen(prefix)) == 0) ? 1U : 0U;
}

static uint8 menu_air_command_count_internal(void)
{
    return s_air_command_count;
}

static void menu_air_command_reset(uint8 keep_ack)
{
    s_air_cmd_status.state = MENU_AIR_CMD_STATE_IDLE;
    s_air_cmd_status.active_index = MENU_AIR_CMD_INVALID_INDEX;
    s_air_cmd_status.mode = MENU_AIR_COMMAND_MODE_POLLING;
    s_air_cmd_status.send_tick = 0U;
    if(keep_ack == 0U)
    {
        s_air_cmd_status.last_result = AIR_COMM_ACK_RESULT_NONE;
        s_air_cmd_status.last_status = AIR_COMM_STATUS_ERROR;
        s_air_cmd_status.last_ack_text[0] = '\0';
    }
}

static void menu_air_command_record_ack(uint8 result, uint8 status)
{
    s_air_cmd_status.last_result = result;
    s_air_cmd_status.last_status = status;
    if(result == AIR_COMM_ACK_RESULT_TIMEOUT)
    {
        s_air_cmd_status.timeout_count++;
        strncpy(s_air_cmd_status.last_ack_text, "Comm Timeout", sizeof(s_air_cmd_status.last_ack_text) - 1U);
        s_air_cmd_status.last_ack_text[sizeof(s_air_cmd_status.last_ack_text) - 1U] = '\0';
    }
    else
    {
        (void)air_comm_car_get_last_command_ack_text(s_air_cmd_status.last_ack_text,
                                                     (uint8)sizeof(s_air_cmd_status.last_ack_text));
    }
}

static void menu_air_command_timeout(void)
{
    s_air_cmd_status.timeout_count++;
    s_air_cmd_status.last_result = AIR_COMM_ACK_RESULT_TIMEOUT;
    s_air_cmd_status.last_status = AIR_COMM_STATUS_ERROR;
    strncpy(s_air_cmd_status.last_ack_text, "Comm Timeout", sizeof(s_air_cmd_status.last_ack_text) - 1U);
    s_air_cmd_status.last_ack_text[sizeof(s_air_cmd_status.last_ack_text) - 1U] = '\0';
    air_comm_car_cancel_pending_command();
    menu_air_command_reset(1U);
    menu_request_refresh(REFRESH_FULL);
}

static void menu_air_sync_reset(uint8 mode)
{
    s_air_sync_status.sending = 0U;
    s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;
    s_air_sync_status.mode = mode;
    s_air_sync_next_index = 0U;
}

static void menu_air_clear_dirty(void)
{
    memset(s_air_param_dirty, 0, sizeof(s_air_param_dirty));
    s_air_sync_status.dirty_count = 0U;
}

static void menu_air_record_failure(uint8 index, uint8 result, uint8 status)
{
    s_air_sync_status.last_failed_index = index;
    s_air_sync_status.last_result = result;
    s_air_sync_status.last_status = status;
    s_air_sync_status.fail_count++;
    if(result == AIR_COMM_ACK_RESULT_TIMEOUT)
    {
        s_air_sync_status.timeout_count++;
    }
}

static void menu_air_register_fc_params(void)
{
    s_air_param_values[0] = 0.001f;
    s_air_param_values[1] = 0.002f;
    s_air_param_values[2] = 0.02f;
    s_air_param_values[3] = 0.02f;
    s_air_param_values[4] = 0.02f;
    s_air_param_values[5] = 0.01f;
    s_air_param_values[6] = 3950.0f;
    s_air_param_values[7] = 0.33f;
    s_air_param_values[8] = -0.40f;
    s_air_param_values[9] = 5.0f;
    s_air_param_values[10] = 0.0f;
    s_air_param_values[11] = 0.0f;
    s_air_param_values[12] = 0.0f;
    s_air_param_values[13] = 180.0f;
    s_air_param_values[14] = 30.0f;
    s_air_param_values[15] = 6.0f;
    s_air_param_values[16] = 0.10f;
    s_air_param_values[17] = 0.0f;
    s_air_param_values[18] = 0.0f;
    s_air_param_values[19] = 100.0f;
    s_air_param_values[20] = 30.0f;
    s_air_param_values[21] = 14.0f;
    s_air_param_values[22] = 4.0f;
    s_air_param_values[23] = 0.0f;
    s_air_param_values[24] = 0.0f;
    s_air_param_values[25] = 700.0f;
    s_air_param_values[26] = 30.0f;
    s_air_param_values[27] = 6.0f;
    s_air_param_values[28] = 0.0f;
    s_air_param_values[29] = 0.0f;
    s_air_param_values[30] = 0.10f;
    s_air_param_values[31] = 80.0f;
    s_air_param_values[32] = 15.0f;
    s_air_param_values[33] = 6.0f;
    s_air_param_values[34] = 0.0f;
    s_air_param_values[35] = 0.0f;
    s_air_param_values[36] = 0.10f;
    s_air_param_values[37] = 80.0f;
    s_air_param_values[38] = 15.0f;
    s_air_param_values[39] = 1.5f;
    s_air_param_values[40] = 0.0f;
    s_air_param_values[41] = 0.0f;
    s_air_param_values[42] = 0.0f;
    s_air_param_values[43] = 0.0f;
    s_air_param_values[44] = 0.0f;
    s_air_param_values[45] = 0.90f;
    s_air_param_values[46] = 0.0f;
    s_air_param_values[47] = 0.0f;
    s_air_param_values[48] = 0.0f;
    s_air_param_values[49] = 0.0f;
    s_air_param_values[50] = 0.0f;
    s_air_param_values[51] = 0.90f;
    s_air_param_values[52] = 0.0f;
    s_air_param_values[53] = 0.0f;
    s_air_param_values[54] = 0.0f;
    s_air_param_values[55] = 0.0f;
    s_air_param_values[56] = 0.0f;
    s_air_param_values[57] = 1.0f;
    s_air_param_values[58] = 0.0f;
    s_air_param_values[59] = 0.06f;
    s_air_param_values[60] = 0.0f;
    s_air_param_values[61] = 0.0f;
    s_air_param_values[62] = 0.2f;
    s_air_param_values[63] = 0.13f;
    s_air_param_values[64] = 0.024f;
    s_air_param_values[65] = 0.0f;
    s_air_param_values[66] = 0.0f;
    s_air_param_values[67] = 3.0f;
    s_air_param_values[68] = 0.0f;
    s_air_param_values[69] = 0.13f;
    s_air_param_values[70] = 0.024f;
    s_air_param_values[71] = 0.0f;
    s_air_param_values[72] = 0.0f;
    s_air_param_values[73] = 3.0f;
    s_air_param_values[74] = 0.0f;
    s_air_param_values[75] = 900.0f;
    s_air_param_values[76] = 95.0f;
    s_air_param_values[77] = 0.0f;
    s_air_param_values[78] = 0.0f;
    s_air_param_values[79] = 900.0f;
    s_air_param_values[80] = 0.0f;
    s_air_param_values[81] = 0.06f;
    s_air_param_values[82] = 0.18f;
    s_air_param_values[83] = 10.0f;
    s_air_param_values[84] = 0.04f;
    s_air_param_values[85] = 0.7f;
    s_air_param_values[86] = 0.0f;
    s_air_param_values[87] = 0.0f;
    s_air_param_values[88] = 0.0f;
    s_air_param_values[89] = 0.0f;
    s_air_param_values[90] = 0.0f;
    s_air_param_values[91] = 0.7f;
    s_air_param_values[92] = 0.0f;
    s_air_param_values[93] = 0.0f;
    s_air_param_values[94] = 0.0f;
    s_air_param_values[95] = 0.0f;
    s_air_param_values[96] = 0.0f;

    menu_register_param_air("gyro_dt", &s_air_param_values[0], 0.0001f, 0.0001f, 0.1f);
    menu_register_param_air("angle_dt", &s_air_param_values[1], 0.0001f, 0.0001f, 0.1f);
    menu_register_param_air("pos_xy_dt", &s_air_param_values[2], 0.001f, 0.0001f, 0.2f);
    menu_register_param_air("pos_z_dt", &s_air_param_values[3], 0.001f, 0.0001f, 0.2f);
    menu_register_param_air("vel_xy_dt", &s_air_param_values[4], 0.001f, 0.0001f, 0.2f);
    menu_register_param_air("vel_z_dt", &s_air_param_values[5], 0.001f, 0.0001f, 0.2f);
    menu_register_param_air("base_throttle", &s_air_param_values[6], 10.0f, 0.0f, 6000.0f);
    menu_register_param_air("roll_mech_trim_deg", &s_air_param_values[7], 0.01f, -30.0f, 30.0f);
    menu_register_param_air("pitch_mech_trim_deg", &s_air_param_values[8], 0.01f, -30.0f, 30.0f);

    menu_register_param_air("roll_gyro_kp", &s_air_param_values[9], 0.1f, 0.0f, 3000.0f);
    menu_register_param_air("roll_gyro_ki", &s_air_param_values[10], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("roll_gyro_kd", &s_air_param_values[11], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("roll_gyro_kff", &s_air_param_values[12], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("roll_gyro_i_limit", &s_air_param_values[13], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("roll_gyro_d_lpf", &s_air_param_values[14], 1.0f, 0.0f, 500.0f);
    menu_register_param_air("pitch_gyro_kp", &s_air_param_values[15], 0.1f, 0.0f, 3000.0f);
    menu_register_param_air("pitch_gyro_ki", &s_air_param_values[16], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pitch_gyro_kd", &s_air_param_values[17], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pitch_gyro_kff", &s_air_param_values[18], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pitch_gyro_i_limit", &s_air_param_values[19], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("pitch_gyro_d_lpf", &s_air_param_values[20], 1.0f, 0.0f, 500.0f);
    menu_register_param_air("yaw_gyro_kp", &s_air_param_values[21], 0.1f, 0.0f, 3000.0f);
    menu_register_param_air("yaw_gyro_ki", &s_air_param_values[22], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("yaw_gyro_kd", &s_air_param_values[23], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("yaw_gyro_kff", &s_air_param_values[24], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("yaw_gyro_i_limit", &s_air_param_values[25], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("yaw_gyro_d_lpf", &s_air_param_values[26], 1.0f, 0.0f, 500.0f);

    menu_register_param_air("roll_angle_kp", &s_air_param_values[27], 0.1f, 0.0f, 3000.0f);
    menu_register_param_air("roll_angle_ki", &s_air_param_values[28], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("roll_angle_kd", &s_air_param_values[29], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("roll_angle_kff", &s_air_param_values[30], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("roll_angle_i_limit", &s_air_param_values[31], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("roll_angle_d_lpf", &s_air_param_values[32], 1.0f, 0.0f, 500.0f);
    menu_register_param_air("pitch_angle_kp", &s_air_param_values[33], 0.1f, 0.0f, 3000.0f);
    menu_register_param_air("pitch_angle_ki", &s_air_param_values[34], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pitch_angle_kd", &s_air_param_values[35], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pitch_angle_kff", &s_air_param_values[36], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pitch_angle_i_limit", &s_air_param_values[37], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("pitch_angle_d_lpf", &s_air_param_values[38], 1.0f, 0.0f, 500.0f);
    menu_register_param_air("yaw_angle_kp", &s_air_param_values[39], 0.1f, 0.0f, 3000.0f);
    menu_register_param_air("yaw_angle_ki", &s_air_param_values[40], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("yaw_angle_kd", &s_air_param_values[41], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("yaw_angle_kff", &s_air_param_values[42], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("yaw_angle_i_limit", &s_air_param_values[43], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("yaw_angle_d_lpf", &s_air_param_values[44], 1.0f, 0.0f, 500.0f);

    menu_register_param_air("pos_x_kp", &s_air_param_values[45], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_x_ki", &s_air_param_values[46], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_x_kd", &s_air_param_values[47], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_x_kff", &s_air_param_values[48], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_x_i_limit", &s_air_param_values[49], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("pos_x_d_lpf", &s_air_param_values[50], 0.1f, 0.0f, 500.0f);
    menu_register_param_air("pos_y_kp", &s_air_param_values[51], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_y_ki", &s_air_param_values[52], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_y_kd", &s_air_param_values[53], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_y_kff", &s_air_param_values[54], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_y_i_limit", &s_air_param_values[55], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("pos_y_d_lpf", &s_air_param_values[56], 0.1f, 0.0f, 500.0f);
    menu_register_param_air("pos_z_kp", &s_air_param_values[57], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_z_ki", &s_air_param_values[58], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_z_kd", &s_air_param_values[59], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_z_kff", &s_air_param_values[60], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("pos_z_i_limit", &s_air_param_values[61], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("pos_z_d_lpf", &s_air_param_values[62], 0.1f, 0.0f, 500.0f);

    menu_register_param_air("vel_x_kp", &s_air_param_values[63], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("vel_x_ki", &s_air_param_values[64], 0.001f, 0.0f, 3000.0f);
    menu_register_param_air("vel_x_kd", &s_air_param_values[65], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("vel_x_kff", &s_air_param_values[66], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("vel_x_i_limit", &s_air_param_values[67], 0.1f, 0.0f, 5000.0f);
    menu_register_param_air("vel_x_d_lpf", &s_air_param_values[68], 0.1f, 0.0f, 500.0f);
    menu_register_param_air("vel_y_kp", &s_air_param_values[69], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("vel_y_ki", &s_air_param_values[70], 0.001f, 0.0f, 3000.0f);
    menu_register_param_air("vel_y_kd", &s_air_param_values[71], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("vel_y_kff", &s_air_param_values[72], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("vel_y_i_limit", &s_air_param_values[73], 0.1f, 0.0f, 5000.0f);
    menu_register_param_air("vel_y_d_lpf", &s_air_param_values[74], 0.1f, 0.0f, 500.0f);
    menu_register_param_air("vel_z_kp", &s_air_param_values[75], 1.0f, 0.0f, 3000.0f);
    menu_register_param_air("vel_z_ki", &s_air_param_values[76], 1.0f, 0.0f, 3000.0f);
    menu_register_param_air("vel_z_kd", &s_air_param_values[77], 0.1f, 0.0f, 3000.0f);
    menu_register_param_air("vel_z_kff", &s_air_param_values[78], 0.1f, 0.0f, 3000.0f);
    menu_register_param_air("vel_z_i_limit", &s_air_param_values[79], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("vel_z_d_lpf", &s_air_param_values[80], 0.1f, 0.0f, 500.0f);

    menu_register_param_air("mode1_track_ff_deg_per_cmps", &s_air_param_values[81], 0.001f, 0.0f, 1.0f);
    menu_register_param_air("mode1_brake_kp", &s_air_param_values[82], 0.01f, 0.0f, 50.0f);
    menu_register_param_air("mode1_brake_exit_vel_cmps", &s_air_param_values[83], 1.0f, 0.0f, 300.0f);
    menu_register_param_air("pos_est_k_flow", &s_air_param_values[84], 0.001f, 0.0f, 1.0f);

    menu_register_param_air("mode8_img_x_kp", &s_air_param_values[85], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("mode8_img_x_ki", &s_air_param_values[86], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("mode8_img_x_kd", &s_air_param_values[87], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("mode8_img_x_kff", &s_air_param_values[88], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("mode8_img_x_i_limit", &s_air_param_values[89], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("mode8_img_x_d_lpf", &s_air_param_values[90], 0.1f, 0.0f, 500.0f);
    menu_register_param_air("mode8_img_y_kp", &s_air_param_values[91], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("mode8_img_y_ki", &s_air_param_values[92], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("mode8_img_y_kd", &s_air_param_values[93], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("mode8_img_y_kff", &s_air_param_values[94], 0.01f, 0.0f, 3000.0f);
    menu_register_param_air("mode8_img_y_i_limit", &s_air_param_values[95], 1.0f, 0.0f, 5000.0f);
    menu_register_param_air("mode8_img_y_d_lpf", &s_air_param_values[96], 0.1f, 0.0f, 500.0f);
}

static uint32 menu_air_calc_checksum(uint8 count)
{
    uint8 index;
    uint32 checksum = 0x13572468UL;
    float value;
    uint32 value_bits;

    for(index = 0U; index < count; index++)
    {
        value = *(s_air_params[index].variable);
        memcpy(&value_bits, &value, sizeof(value_bits));
        checksum ^= value_bits;
        checksum = (checksum << 5) | (checksum >> 27);
        checksum += (uint32)(index + 1U) * 2654435761UL;
    }

    return checksum;
}

static uint32 menu_air_calc_buffer_checksum(uint8 count, uint32 offset)
{
    uint8 index;
    uint32 checksum = 0x13572468UL;
    uint32 value_bits;

    for(index = 0U; index < count; index++)
    {
        value_bits = flash_union_buffer[offset + index].uint32_type;
        checksum ^= value_bits;
        checksum = (checksum << 5) | (checksum >> 27);
        checksum += (uint32)(index + 1U) * 2654435761UL;
    }

    return checksum;
}

static uint8 menu_air_slot_valid(uint8 slot, menu_air_slot_header_t *out_header)
{
    uint32 page;
    menu_air_slot_header_t *header;

    if(slot >= MENU_AIR_SLOT_COUNT)
    {
        return 0U;
    }

    page = MENU_AIR_SLOT_BASE_PAGE + ((uint32)slot * MENU_AIR_SLOT_SIZE);
    if((page + MENU_AIR_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        return 0U;
    }

    if(flash_check(0U, page) == 0U)
    {
        return 0U;
    }

    flash_read_page_to_buffer(0U, page, FLASH_PAGE_LENGTH);
    header = (menu_air_slot_header_t *)flash_union_buffer;

    if((header->magic != MENU_AIR_MAGIC_NUMBER) ||
       (header->version != MENU_AIR_VERSION) ||
       (header->slot_id != slot) ||
       (header->param_count > MENU_AIR_MAX_PARAMS))
    {
        return 0U;
    }

    if(header->checksum != menu_air_calc_buffer_checksum(header->param_count,
        (uint32)((sizeof(menu_air_slot_header_t) + 3U) / 4U)))
    {
        return 0U;
    }

    if(out_header != NULL)
    {
        *out_header = *header;
    }

    return 1U;
}

void menu_air_support_init(void)
{
    s_air_param_count = 0U;
    memset(s_air_param_values, 0, sizeof(s_air_param_values));
    memset(s_air_param_dirty, 0, sizeof(s_air_param_dirty));
    memset(&s_air_sync_status, 0, sizeof(s_air_sync_status));
    s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;
    s_air_sync_status.last_failed_index = MENU_AIR_SYNC_INVALID_INDEX;
    s_air_sync_status.mode = MENU_AIR_SYNC_MODE_IDLE;
    s_air_sync_status.reason = MENU_AIR_SYNC_REASON_NONE;
    s_air_sync_next_index = 0U;
    s_air_last_online = 0U;
    s_air_boot_sync_done = 0U;
    s_air_pending_sync_reason = MENU_AIR_SYNC_REASON_NONE;
    memset(&s_air_cmd_status, 0, sizeof(s_air_cmd_status));
    s_air_cmd_status.active_index = MENU_AIR_CMD_INVALID_INDEX;
    s_air_cmd_status.state = MENU_AIR_CMD_STATE_IDLE;
    s_air_cmd_status.last_status = AIR_COMM_STATUS_ERROR;
    memset(s_air_commands, 0, sizeof(s_air_commands));
    s_air_command_count = 0U;

    menu_air_register_fc_params();
    menu_air_register_default_commands();

    if(menu_air_slot_valid(0U, NULL) != 0U)
    {
        (void)menu_load_air_slot_internal(0U, 0U);
        menu_air_clear_dirty();
        menu_air_sync_reset(MENU_AIR_SYNC_MODE_IDLE);
        s_air_sync_status.reason = MENU_AIR_SYNC_REASON_NONE;
        s_air_pending_sync_reason = MENU_AIR_SYNC_REASON_NONE;
    }
}

uint8 menu_air_register_polling_command(const char *name)
{
    return menu_air_register_command_internal(name, MENU_AIR_COMMAND_MODE_POLLING);
}

uint8 menu_air_register_instant_command(const char *name)
{
    return menu_air_register_command_internal(name, MENU_AIR_COMMAND_MODE_INSTANT);
}

void menu_register_param_air(const char *name, float *var, float step, float min, float max)
{
    uint8 name_len;

    if((name == NULL) || (var == NULL) || (s_air_param_count >= MENU_AIR_MAX_PARAMS))
    {
        return;
    }

    name_len = (uint8)strlen(name);
    if((name_len == 0U) || (name_len >= sizeof(s_air_params[0].name)))
    {
        return;
    }

    memset(s_air_params[s_air_param_count].name, 0, sizeof(s_air_params[s_air_param_count].name));
    memcpy(s_air_params[s_air_param_count].name, name, name_len);
    s_air_params[s_air_param_count].variable = var;
    s_air_params[s_air_param_count].step = step;
    s_air_params[s_air_param_count].min_val = min;
    s_air_params[s_air_param_count].max_val = max;
    *(s_air_params[s_air_param_count].variable) =
        car_math_clampf(*(s_air_params[s_air_param_count].variable), min, max);
    s_air_param_count++;
}

uint8 menu_get_air_param_count(void)
{
    return s_air_param_count;
}

float menu_get_air_param_by_index(uint8 index)
{
    if((index >= s_air_param_count) || (s_air_params[index].variable == NULL))
    {
        return 0.0f;
    }

    return *(s_air_params[index].variable);
}

uint8 menu_set_air_param_by_index(uint8 index, float value)
{
    if((index >= s_air_param_count) || (s_air_params[index].variable == NULL))
    {
        return 1U;
    }

    if(menu_can_edit_air_params() == 0U)
    {
        /* AIR离线或车端运行时禁止修改AIR参数影子值，避免离线编辑后误同步。 */
        return 1U;
    }

    value = car_math_clampf(value, s_air_params[index].min_val, s_air_params[index].max_val);
    *(s_air_params[index].variable) = value;

    return 0U;
}

const menu_air_param_config_t *menu_get_air_param_config(uint8 index)
{
    if(index >= s_air_param_count)
    {
        return NULL;
    }

    return &s_air_params[index];
}

uint8 menu_is_air_connected(void)
{
    return air_comm_car_is_online();
}

uint8 menu_can_edit_air_params(void)
{
    if(menu_is_air_connected() == 0U)
    {
        return 0U;
    }

    return ((car_control_enabled == 0U) ||
            (car_emergency_stop_active != 0U)) ? 1U : 0U;
}

uint8 menu_sync_all_air_params(void)
{
    return menu_air_sync_all_start(MENU_AIR_SYNC_REASON_MANUAL);
}

uint8 menu_air_sync_all_start(uint8 reason)
{
    if((menu_can_edit_air_params() == 0U) ||
       (s_air_param_count == 0U) ||
       (s_air_sync_status.sending != 0U) ||
       (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_COMMIT) ||
       (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_FULL) ||
       (air_comm_car_has_pending_ack() != 0U))
    {
        if(reason == MENU_AIR_SYNC_REASON_MANUAL)
        {
            if(menu_is_air_connected() == 0U)
            {
                menu_show_error("Air Offline");
            }
            else if((car_control_enabled != 0U) && (car_emergency_stop_active == 0U))
            {
                menu_show_error("Car Active");
            }
            else
            {
                menu_show_error("Air Busy");
            }
        }
        return 1U;
    }

    menu_air_sync_reset(MENU_AIR_SYNC_MODE_FULL);
    s_air_sync_status.reason = reason;
    s_air_sync_status.dirty_count = s_air_param_count;
    s_air_sync_status.last_result = AIR_COMM_ACK_RESULT_NONE;
    s_air_sync_status.last_status = AIR_COMM_STATUS_ERROR;
    s_air_pending_sync_reason = MENU_AIR_SYNC_REASON_NONE;

    return 0U;
}

uint8 menu_air_is_busy(void)
{
    return ((s_air_sync_status.sending != 0U) ||
            (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_COMMIT) ||
            (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_FULL) ||
            (s_air_cmd_status.state != MENU_AIR_CMD_STATE_IDLE) ||
            (air_comm_car_has_pending_ack() != 0U)) ? 1U : 0U;
}

void menu_air_stop_param_sync(void)
{
    if((s_air_sync_status.mode == MENU_AIR_SYNC_MODE_COMMIT) ||
       (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_FULL) ||
       (s_air_sync_status.sending != 0U))
    {
        menu_air_record_failure(s_air_sync_status.active_index,
                                AIR_COMM_ACK_RESULT_TIMEOUT,
                                AIR_COMM_STATUS_ERROR);
        menu_air_sync_reset(MENU_AIR_SYNC_MODE_FAIL);
    }

    air_comm_car_cancel_pending_set_param();
}

uint8 menu_air_commit_param(uint8 index)
{
    if((index >= s_air_param_count) || (s_air_params[index].variable == NULL))
    {
        return 1U;
    }

    return menu_air_commit_param_value(index, *(s_air_params[index].variable));
}

uint8 menu_air_commit_param_value(uint8 index, float value)
{
    uint32 start_ms;
    uint8 ack_type = 0U;
    uint8 ack_result = AIR_COMM_ACK_RESULT_NONE;
    uint8 ack_status = AIR_COMM_STATUS_ERROR;

    if((index >= s_air_param_count) || (s_air_params[index].variable == NULL))
    {
        return 1U;
    }

    value = car_math_clampf(value, s_air_params[index].min_val, s_air_params[index].max_val);

    if(menu_can_edit_air_params() == 0U)
    {
        menu_show_error((menu_is_air_connected() == 0U) ?
                        "Air Offline" : "Car Active");
        return 1U;
    }

    if(menu_air_is_busy() != 0U)
    {
        menu_show_error("Air Busy");
        return 1U;
    }

    menu_show_progress("Syncing");
    if(air_comm_car_set_param(s_air_params[index].name, value) != 0U)
    {
        menu_air_record_failure(index, AIR_COMM_ACK_RESULT_ERROR, AIR_COMM_STATUS_ERROR);
        menu_show_error("Air Send Fail");
        return 1U;
    }

    s_air_sync_status.sending = 1U;
    s_air_sync_status.active_index = index;
    s_air_sync_status.mode = MENU_AIR_SYNC_MODE_COMMIT;
    s_air_sync_status.reason = MENU_AIR_SYNC_REASON_COMMIT;
    s_air_sync_status.last_result = AIR_COMM_ACK_RESULT_NONE;
    s_air_sync_status.last_status = AIR_COMM_STATUS_ERROR;
    s_air_sync_status.send_count++;

    start_ms = air_comm_car_get_tick();
    while((air_comm_car_get_tick() - start_ms) < MENU_AIR_COMMIT_WAIT_MS)
    {
        air_comm_car_poll();
        air_comm_car_update_100HZ();
        (void)air_comm_car_get_last_ack(&ack_type, &ack_result, &ack_status);
        if(air_comm_car_has_pending_ack() != 0U)
        {
            continue;
        }

        if((ack_type == MENU_AIR_ACK_TYPE_SET_PARAM) &&
           (ack_result != AIR_COMM_ACK_RESULT_NONE))
        {
            break;
        }

        ack_result = AIR_COMM_ACK_RESULT_ERROR;
        ack_status = AIR_COMM_STATUS_ERROR;
        break;
    }

    if(air_comm_car_has_pending_ack() != 0U)
    {
        air_comm_car_cancel_pending_set_param();
    }

    (void)air_comm_car_get_last_ack(&ack_type, &ack_result, &ack_status);
    s_air_sync_status.sending = 0U;
    s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;
    s_air_sync_status.last_result = ack_result;
    s_air_sync_status.last_status = ack_status;
    s_air_sync_status.mode = MENU_AIR_SYNC_MODE_IDLE;
    s_air_sync_status.dirty_count = menu_air_dirty_count();

    if((ack_type == MENU_AIR_ACK_TYPE_SET_PARAM) &&
       (menu_air_ack_is_success(ack_result, ack_status) != 0U))
    {
        (void)air_comm_car_get_last_ack_value(s_air_params[index].variable);
        s_air_param_dirty[index] = 0U;
        s_air_sync_status.ok_count++;
        menu_show_success("Air OK");
        return 0U;
    }

    if(ack_result == AIR_COMM_ACK_RESULT_TIMEOUT)
    {
        menu_air_record_failure(index, ack_result, ack_status);
        menu_show_error("Air Timeout");
    }
    else
    {
        menu_air_record_failure(index, ack_result, ack_status);
        menu_show_error("Air Fail");
    }

    return 1U;
}

void menu_air_update_100HZ(void)
{
    uint8 ack_type = 0U;
    uint8 ack_result = AIR_COMM_ACK_RESULT_NONE;
    uint8 ack_status = AIR_COMM_STATUS_ERROR;
    uint8 active_index;
    uint8 online = menu_is_air_connected();
    uint8 sync_reason = s_air_pending_sync_reason;

    if(s_air_cmd_status.state != MENU_AIR_CMD_STATE_IDLE)
    {
        return;
    }

    if((online != 0U) && (s_air_last_online == 0U) && (s_air_boot_sync_done == 0U))
    {
        sync_reason = MENU_AIR_SYNC_REASON_BOOT;
    }

    if((online != 0U) &&
       (sync_reason != MENU_AIR_SYNC_REASON_NONE) &&
       (s_air_sync_status.mode != MENU_AIR_SYNC_MODE_FULL) &&
       (s_air_sync_status.sending == 0U) &&
       (air_comm_car_has_pending_ack() == 0U))
    {
        if(menu_air_sync_all_start(sync_reason) == 0U)
        {
            if(sync_reason == MENU_AIR_SYNC_REASON_BOOT)
            {
                s_air_boot_sync_done = 1U;
            }
            s_air_pending_sync_reason = MENU_AIR_SYNC_REASON_NONE;
        }
    }
    s_air_last_online = online;

    if(s_air_sync_status.sending != 0U)
    {
        (void)air_comm_car_get_last_ack(&ack_type, &ack_result, &ack_status);
        if(air_comm_car_has_pending_ack() != 0U)
        {
            return;
        }

        if((ack_type == MENU_AIR_ACK_TYPE_SET_PARAM) &&
           (ack_result != AIR_COMM_ACK_RESULT_NONE))
        {
            active_index = s_air_sync_status.active_index;
            s_air_sync_status.last_result = ack_result;
            s_air_sync_status.last_status = ack_status;
            s_air_sync_status.sending = 0U;
            s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;

            if((active_index < s_air_param_count) &&
               (menu_air_ack_is_success(ack_result, ack_status) != 0U))
            {
                (void)air_comm_car_get_last_ack_value(s_air_params[active_index].variable);
                s_air_param_dirty[active_index] = 0U;
                s_air_sync_status.ok_count++;
                if(s_air_sync_status.dirty_count > 0U)
                {
                    s_air_sync_status.dirty_count--;
                }
            }
            else
            {
                if(active_index < s_air_param_count)
                {
                    s_air_param_dirty[active_index] = 0U;
                }
                menu_air_record_failure(active_index, ack_result, ack_status);
                menu_air_sync_reset(MENU_AIR_SYNC_MODE_FAIL);
                return;
            }
        }
        else
        {
            active_index = s_air_sync_status.active_index;
            s_air_sync_status.sending = 0U;
            s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;
            menu_air_record_failure(active_index,
                                    AIR_COMM_ACK_RESULT_ERROR,
                                    AIR_COMM_STATUS_ERROR);
            menu_air_sync_reset(MENU_AIR_SYNC_MODE_FAIL);
        }

        return;
    }

    if(s_air_sync_status.mode != MENU_AIR_SYNC_MODE_FULL)
    {
        s_air_sync_status.dirty_count = menu_air_dirty_count();
        return;
    }

    if((menu_can_edit_air_params() == 0U) || (air_comm_car_has_pending_ack() != 0U))
    {
        return;
    }

    if(s_air_sync_next_index >= s_air_param_count)
    {
        menu_air_sync_reset(MENU_AIR_SYNC_MODE_DONE);
        s_air_sync_status.dirty_count = 0U;
        return;
    }

    if(air_comm_car_set_param(s_air_params[s_air_sync_next_index].name,
                              *(s_air_params[s_air_sync_next_index].variable)) == 0U)
    {
        s_air_sync_status.sending = 1U;
        s_air_sync_status.active_index = s_air_sync_next_index;
        s_air_sync_status.last_result = AIR_COMM_ACK_RESULT_NONE;
        s_air_sync_status.last_status = AIR_COMM_STATUS_ERROR;
        s_air_sync_status.send_count++;
        s_air_sync_next_index++;
    }
    else
    {
        menu_air_record_failure(s_air_sync_next_index,
                                AIR_COMM_ACK_RESULT_ERROR,
                                AIR_COMM_STATUS_ERROR);
        menu_air_sync_reset(MENU_AIR_SYNC_MODE_FAIL);
    }
}

void menu_get_air_sync_status(menu_air_sync_status_t *status)
{
    if(status == NULL)
    {
        return;
    }

    if((s_air_sync_status.mode != MENU_AIR_SYNC_MODE_FULL) &&
       (s_air_sync_status.sending == 0U))
    {
        s_air_sync_status.dirty_count = menu_air_dirty_count();
    }
    *status = s_air_sync_status;
}

uint8 menu_air_command_get_count(void)
{
    return menu_air_command_count_internal();
}

const char *menu_air_command_get_name(uint8 index)
{
    if(index >= menu_air_command_count_internal())
    {
        return "";
    }

    return s_air_commands[index].name;
}

uint8 menu_air_command_is_active(void)
{
    return (s_air_cmd_status.state != MENU_AIR_CMD_STATE_IDLE) ? 1U : 0U;
}

uint8 menu_air_command_is_running(uint8 index)
{
    return ((s_air_cmd_status.state != MENU_AIR_CMD_STATE_IDLE) &&
            (s_air_cmd_status.active_index == index)) ? 1U : 0U;
}

uint8 menu_air_command_start(uint8 index)
{
    if(index >= menu_air_command_count_internal())
    {
        menu_show_error("Cmd Err");
        return 1U;
    }

    if(menu_is_air_connected() == 0U)
    {
        menu_show_error("Air Offline");
        return 1U;
    }

    if(menu_air_is_busy() != 0U)
    {
        menu_show_error("Air Busy");
        return 1U;
    }

    air_comm_car_clear_last_ack();
    if(air_comm_car_exec_command(s_air_commands[index].name) != 0U)
    {
        s_air_cmd_status.last_result = AIR_COMM_ACK_RESULT_ERROR;
        s_air_cmd_status.last_status = AIR_COMM_STATUS_ERROR;
        strncpy(s_air_cmd_status.last_ack_text, "Send Fail", sizeof(s_air_cmd_status.last_ack_text) - 1U);
        s_air_cmd_status.last_ack_text[sizeof(s_air_cmd_status.last_ack_text) - 1U] = '\0';
        menu_show_error("Send Fail");
        return 1U;
    }

    s_air_cmd_status.state = MENU_AIR_CMD_STATE_WAIT_START_ACK;
    s_air_cmd_status.active_index = index;
    s_air_cmd_status.mode = s_air_commands[index].mode;
    s_air_cmd_status.last_result = AIR_COMM_ACK_RESULT_NONE;
    s_air_cmd_status.last_status = AIR_COMM_STATUS_ERROR;
    s_air_cmd_status.last_ack_text[0] = '\0';
    s_air_cmd_status.send_tick = air_comm_car_get_tick();
    menu_request_refresh(REFRESH_FULL);

    return 0U;
}

uint8 menu_air_command_stop(void)
{
    if(s_air_cmd_status.state == MENU_AIR_CMD_STATE_IDLE)
    {
        return 1U;
    }

    if(air_comm_car_has_pending_ack() != 0U)
    {
        return 1U;
    }

    air_comm_car_clear_last_ack();
    if(air_comm_car_exec_command("NONE") != 0U)
    {
        s_air_cmd_status.last_result = AIR_COMM_ACK_RESULT_ERROR;
        s_air_cmd_status.last_status = AIR_COMM_STATUS_ERROR;
        strncpy(s_air_cmd_status.last_ack_text, "Stop Fail", sizeof(s_air_cmd_status.last_ack_text) - 1U);
        s_air_cmd_status.last_ack_text[sizeof(s_air_cmd_status.last_ack_text) - 1U] = '\0';
        menu_show_error("Stop Fail");
        return 1U;
    }

    s_air_cmd_status.state = MENU_AIR_CMD_STATE_WAIT_EXIT_ACK;
    s_air_cmd_status.send_tick = air_comm_car_get_tick();
    menu_request_refresh(REFRESH_FULL);

    return 0U;
}

void menu_air_command_update_100HZ(void)
{
    uint8 ack_type = 0U;
    uint8 ack_result = AIR_COMM_ACK_RESULT_NONE;
    uint8 ack_status = AIR_COMM_STATUS_ERROR;

    if(s_air_cmd_status.state == MENU_AIR_CMD_STATE_IDLE)
    {
        return;
    }

    if((air_comm_car_get_tick() - s_air_cmd_status.send_tick) >= MENU_AIR_COMMAND_TIMEOUT_MS)
    {
        if((s_air_cmd_status.state == MENU_AIR_CMD_STATE_WAIT_START_ACK) ||
           (s_air_cmd_status.state == MENU_AIR_CMD_STATE_WAIT_EXIT_ACK) ||
           ((s_air_cmd_status.state == MENU_AIR_CMD_STATE_INSTANT_RUNNING) &&
            (s_air_cmd_status.mode == MENU_AIR_COMMAND_MODE_INSTANT)))
        {
            menu_air_command_timeout();
            return;
        }
    }

    (void)air_comm_car_get_last_ack(&ack_type, &ack_result, &ack_status);
    if((ack_type != MENU_AIR_ACK_TYPE_COMMAND) ||
       (ack_result == AIR_COMM_ACK_RESULT_NONE))
    {
        return;
    }

    menu_air_command_record_ack(ack_result, ack_status);

    if(s_air_cmd_status.state == MENU_AIR_CMD_STATE_WAIT_START_ACK)
    {
        if((ack_result == AIR_COMM_ACK_RESULT_OK) &&
           (ack_status == AIR_COMM_STATUS_OK) &&
           (menu_air_cmd_ack_text_is(s_air_cmd_status.last_ack_text, "ACK_OK") != 0U))
        {
            s_air_cmd_status.state = (s_air_cmd_status.mode == MENU_AIR_COMMAND_MODE_POLLING) ?
                                     MENU_AIR_CMD_STATE_POLLING_RUNNING :
                                     MENU_AIR_CMD_STATE_INSTANT_RUNNING;
            s_air_cmd_status.send_tick = air_comm_car_get_tick();
            air_comm_car_clear_last_ack();
        }
        else
        {
            if(ack_result == AIR_COMM_ACK_RESULT_TIMEOUT)
            {
                air_comm_car_cancel_pending_command();
            }
            menu_air_command_reset(1U);
        }
        menu_request_refresh(REFRESH_FULL);
        return;
    }

    if((s_air_cmd_status.state == MENU_AIR_CMD_STATE_WAIT_EXIT_ACK) ||
       (s_air_cmd_status.state == MENU_AIR_CMD_STATE_INSTANT_RUNNING))
    {
        if((ack_result == AIR_COMM_ACK_RESULT_OK) &&
           (ack_status == AIR_COMM_STATUS_OK) &&
           (menu_air_cmd_ack_text_is(s_air_cmd_status.last_ack_text, "ACK_EXIT_OK") != 0U))
        {
            menu_air_command_reset(1U);
        }
        else if(ack_result != AIR_COMM_ACK_RESULT_NONE)
        {
            menu_air_command_reset(1U);
        }
        menu_request_refresh(REFRESH_FULL);
    }
}

void menu_get_air_command_status(menu_air_cmd_status_t *status)
{
    if(status == NULL)
    {
        return;
    }

    *status = s_air_cmd_status;
}

static uint8 menu_load_air_slot_internal(uint8 slot, uint8 require_online)
{
    menu_air_slot_header_t header;
    uint32 offset;
    uint8 index;
    uint8 count;
    float value;

    if((require_online != 0U) && (menu_can_edit_air_params() == 0U))
    {
        /* 菜单手动加载会改变AIR参数影子值，AIR离线时直接禁止。 */
        menu_show_error((menu_is_air_connected() == 0U) ?
                        "Air Offline" : "Car Active");
        return 1U;
    }

    if(menu_air_is_busy() != 0U)
    {
        menu_show_error("Air Busy");
        return 1U;
    }

    if(menu_air_slot_valid(slot, &header) == 0U)
    {
        menu_show_error("No Air Data");
        return 1U;
    }

    count = (header.param_count < s_air_param_count) ? header.param_count : s_air_param_count;
    offset = (uint32)((sizeof(menu_air_slot_header_t) + 3U) / 4U);

    for(index = 0U; index < count; index++)
    {
        value = flash_union_buffer[offset + index].float_type;
        value = car_math_clampf(value, s_air_params[index].min_val, s_air_params[index].max_val);
        *(s_air_params[index].variable) = value;
    }

    menu_air_clear_dirty();
    if(menu_air_sync_all_start(MENU_AIR_SYNC_REASON_LOAD) != 0U)
    {
        s_air_pending_sync_reason = MENU_AIR_SYNC_REASON_LOAD;
    }

    return 0U;
}

uint8 menu_load_air_slot(uint8 slot)
{
    return menu_load_air_slot_internal(slot, 1U);
}

uint8 menu_save_air_slot(uint8 slot)
{
    uint32 page;
    uint32 offset;
    uint8 index;
    menu_air_slot_header_t *header;

    if(menu_can_edit_air_params() == 0U)
    {
        /* AIR离线时不允许通过菜单保存AIR参数快照，避免离线参数集继续扩散。 */
        menu_show_error((menu_is_air_connected() == 0U) ?
                        "Air Offline" : "Car Active");
        return 1U;
    }

    if(slot >= MENU_AIR_SLOT_COUNT)
    {
        menu_show_error("Air Slot Err");
        return 1U;
    }

    page = MENU_AIR_SLOT_BASE_PAGE + ((uint32)slot * MENU_AIR_SLOT_SIZE);
    if((page + MENU_AIR_SLOT_SIZE - 1U) >= FLASH_PAGE_NUM)
    {
        menu_show_error("Air Page Err");
        return 1U;
    }

    flash_buffer_clear();
    header = (menu_air_slot_header_t *)flash_union_buffer;
    header->magic = MENU_AIR_MAGIC_NUMBER;
    header->version = MENU_AIR_VERSION;
    header->slot_id = slot;
    header->param_count = s_air_param_count;
    header->checksum = menu_air_calc_checksum(s_air_param_count);

    offset = (uint32)((sizeof(menu_air_slot_header_t) + 3U) / 4U);
    for(index = 0U; index < s_air_param_count; index++)
    {
        flash_union_buffer[offset + index].float_type = *(s_air_params[index].variable);
    }

    flash_erase_page(0U, page);
    (void)flash_write_page_from_buffer(0U, page, offset + s_air_param_count);

    return 0U;
}
