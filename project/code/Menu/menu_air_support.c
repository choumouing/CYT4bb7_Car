#include "menu_air_support.h"
#include "menu_config.h"

#define MENU_AIR_SLOT_BASE_PAGE             (80U)
#define MENU_AIR_SLOT_COUNT                 (4U)
#define MENU_AIR_SLOT_SIZE                  (2U)
#define MENU_AIR_MAGIC_NUMBER               (0x41495250UL)
#define MENU_AIR_VERSION                    (4U)
#define MENU_AIR_SYNC_INVALID_INDEX         (0xFFU)
#define MENU_AIR_ACK_TYPE_SET_PARAM         (0x01U)
#define MENU_AIR_ACK_TYPE_COMMAND           (0x03U)
#define MENU_AIR_ACK_TYPE_GET_PARAM         (0x07U)
#define MENU_AIR_COMMAND_TIMEOUT_MS         (1000U)
#define MENU_AIR_PULL_RETRY_MS              (1000U)

#if ((MENU_AIR_BOOT_OVERRIDE_ENABLE != 0U) && (MENU_AIR_BOOT_OVERRIDE_ENABLE != 1U))
#error "Air boot override enable must be 0 or 1"
#endif

#if ((MENU_AIR_BOOT_FLASH_LOAD_ENABLE != 0U) && (MENU_AIR_BOOT_FLASH_LOAD_ENABLE != 1U))
#error "Air boot Flash load enable must be 0 or 1"
#endif

#if (MENU_AIR_BOOT_FLASH_LOAD_SLOT >= MENU_AIR_SLOT_COUNT)
#error "Air boot Flash load slot exceeds menu Air slot count"
#endif

typedef struct
{
    const char *name;
    float default_val;
    float step;
    float min_val;
    float max_val;
    const char *menu_name;
} menu_air_param_definition_t;

typedef struct
{
    uint32 magic;
    uint8 version;
    uint8 slot_id;
    uint16 param_count;
    uint32 checksum;
} menu_air_slot_header_t;

typedef struct
{
    uint32 name_hash;
    float value;
} menu_air_slot_param_t;

typedef char menu_air_slot_param_size_must_be_two_words[
    (sizeof(menu_air_slot_param_t) == (2U * sizeof(uint32))) ? 1 : -1];
typedef char menu_air_slot_data_must_fit_page[
    ((((sizeof(menu_air_slot_header_t) + 3U) / 4U) +
      (MENU_AIR_MAX_PARAMS * (sizeof(menu_air_slot_param_t) / sizeof(uint32)))) <=
     FLASH_PAGE_LENGTH) ? 1 : -1];

static float s_air_param_values[MENU_AIR_MAX_PARAMS];
static menu_air_param_config_t s_air_params[MENU_AIR_MAX_PARAMS];
static uint8 s_air_param_count;
static uint8 s_air_param_dirty[MENU_AIR_MAX_PARAMS];
static menu_air_sync_status_t s_air_sync_status;
static uint8 s_air_sync_next_index;
static uint8 s_air_last_online;
static uint8 s_air_boot_sync_done;
static uint8 s_air_boot_override_done;
static uint8 s_air_catalog_ready;
static uint32 s_air_pull_retry_tick;
static uint8 s_air_deferred_error_reason;
static menu_air_cmd_status_t s_air_cmd_status;

#define MENU_AIR_STRINGIFY_INNER(value)     #value
#define MENU_AIR_STRINGIFY(value)           MENU_AIR_STRINGIFY_INNER(value)
#define MENU_AIR_PARAM(member, default_v, step_v, min_v, max_v, menu_name_v) \
    {MENU_AIR_STRINGIFY(member), (default_v), (step_v), (min_v), (max_v), (menu_name_v)}

static const menu_air_param_definition_t s_air_param_definitions[] =
{
    MENU_AIR_PARAM(gyro_dt, 0.001f,             0.0001f,  0.0001f, 0.1f,    "Basic"),
    MENU_AIR_PARAM(angle_dt, 0.002f,            0.0001f,  0.0001f, 0.1f,    "Basic"),
    MENU_AIR_PARAM(pos_z_dt, 0.02f,            0.001f,   0.0001f, 0.2f,    "Basic"),
    MENU_AIR_PARAM(vel_xy_dt, 0.02f,           0.001f,   0.0001f, 0.2f,    "Basic"),
    MENU_AIR_PARAM(vel_z_dt, 0.01f,            0.001f,   0.0001f, 0.2f,    "Basic"),
    MENU_AIR_PARAM(base_throttle, 3200,      10.0f,     0.0f, 6000.0f,    "Basic"),
    MENU_AIR_PARAM(roll_mech_trim_deg, 0.5f,  0.01f,  -30.0f,   30.0f,    "Basic"),
    MENU_AIR_PARAM(pitch_mech_trim_deg, 1.5f, 0.01f,  -30.0f,   30.0f,    "Basic"),

    MENU_AIR_PARAM(roll_gyro_kp, 5.4f,      0.1f,  0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(roll_gyro_ki, 0.18f,      0.01f, 0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(roll_gyro_kd, 0.010f,      0.01f, 0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(roll_gyro_kff, 0.0f,     0.01f, 0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(roll_gyro_i_limit, 180.0f, 1.0f,  0.0f, 5000.0f, "Gyro PID"),
    MENU_AIR_PARAM(roll_gyro_d_lpf, 60.0f,   1.0f,  0.0f,  500.0f, "Gyro PID"),
    MENU_AIR_PARAM(pitch_gyro_kp, 5.3f,      0.1f,  0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(pitch_gyro_ki, 0.14f,      0.01f, 0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(pitch_gyro_kd, 0.010f,      0.01f, 0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(pitch_gyro_kff, 0.0f,     0.01f, 0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(pitch_gyro_i_limit, 140.0f, 1.0f,  0.0f, 5000.0f, "Gyro PID"),
    MENU_AIR_PARAM(pitch_gyro_d_lpf, 60.0f,   1.0f,  0.0f,  500.0f, "Gyro PID"),
    MENU_AIR_PARAM(yaw_gyro_kp, 15.0f,      0.1f,  0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(yaw_gyro_ki, 5.0f,      0.01f, 0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(yaw_gyro_kd, 0.0f,      0.01f, 0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(yaw_gyro_kff, 0.0f,     0.01f, 0.0f, 3000.0f, "Gyro PID"),
    MENU_AIR_PARAM(yaw_gyro_i_limit, 700.0f, 1.0f,  0.0f, 5000.0f, "Gyro PID"),
    MENU_AIR_PARAM(yaw_gyro_d_lpf, 30.0f,   1.0f,  0.0f,  500.0f, "Gyro PID"),

    MENU_AIR_PARAM(roll_angle_kp, 6.0f,      0.1f,  0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(roll_angle_ki, 0.0f,      0.01f, 0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(roll_angle_kd, 0.0f,      0.01f, 0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(roll_angle_kff, 0.02f,     0.01f, 0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(roll_angle_i_limit, 80.0f, 1.0f,  0.0f, 5000.0f, "Angle PID"),
    MENU_AIR_PARAM(roll_angle_d_lpf, 15.0f,   1.0f,  0.0f,  500.0f, "Angle PID"),
    MENU_AIR_PARAM(pitch_angle_kp, 6.2f,      0.1f,  0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(pitch_angle_ki, 0.0f,      0.01f, 0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(pitch_angle_kd, 0.0f,      0.01f, 0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(pitch_angle_kff, 0.04f,     0.01f, 0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(pitch_angle_i_limit, 80.0f, 1.0f,  0.0f, 5000.0f, "Angle PID"),
    MENU_AIR_PARAM(pitch_angle_d_lpf, 15.0f,   1.0f,  0.0f,  500.0f, "Angle PID"),
    MENU_AIR_PARAM(yaw_angle_kp, 6.0f,      0.1f,  0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(yaw_angle_ki, 0.0f,      0.01f, 0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(yaw_angle_kd, 0.0f,      0.01f, 0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(yaw_angle_kff, 0.0f,     0.01f, 0.0f, 3000.0f, "Angle PID"),
    MENU_AIR_PARAM(yaw_angle_i_limit, 0.0f, 1.0f,  0.0f, 5000.0f, "Angle PID"),
    MENU_AIR_PARAM(yaw_angle_d_lpf, 0.0f,   1.0f,  0.0f,  500.0f, "Angle PID"),

    MENU_AIR_PARAM(vel_x_kp, 0.15f,      0.01f,  0.0f, 3000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_x_ki, 0.02f,      0.001f, 0.0f, 3000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_x_kd, 0.0f,      0.01f,  0.0f, 3000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_x_kff, 0.0f,     0.01f,  0.0f, 3000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_x_i_limit, 3.0f, 0.1f,   0.0f, 5000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_x_d_lpf, 0.0f,   0.1f,   0.0f,  500.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_y_kp, 0.15f,      0.01f,  0.0f, 3000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_y_ki, 0.02f,      0.001f, 0.0f, 3000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_y_kd, 0.0f,      0.01f,  0.0f, 3000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_y_kff, 0.0f,     0.01f,  0.0f, 3000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_y_i_limit, 3.0f, 0.1f,   0.0f, 5000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_y_d_lpf, 0.0f,   0.1f,   0.0f,  500.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_z_ki, 80.0f,      1.0f, 0.0f, 3000.0f, "Vel PID"),
    MENU_AIR_PARAM(vel_z_i_limit, 450.0f, 1.0f, 0.0f, 5000.0f, "Vel PID"),

    MENU_AIR_PARAM(mode7_vel_x_kp, 0.15f,      0.01f,  0.0f, 3000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_x_ki, 0.0f,      0.001f, 0.0f, 3000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_x_kd, 0.0f,      0.01f,  0.0f, 3000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_x_kff, 0.010f,     0.001f, 0.0f, 3000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_x_i_limit, 3.0f, 0.1f,   0.0f, 5000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_x_d_lpf, 0.0f,   0.1f,   0.0f,  500.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_y_kp, 0.15f,      0.01f,  0.0f, 3000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_y_ki, 0.0f,      0.001f, 0.0f, 3000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_y_kd, 0.0f,      0.01f,  0.0f, 3000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_y_kff, 0.010f,     0.001f, 0.0f, 3000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_y_i_limit, 3.0f, 0.1f,   0.0f, 5000.0f, "Mode7"),
    MENU_AIR_PARAM(mode7_vel_y_d_lpf, 0.0f,   0.1f,   0.0f,  500.0f, "Mode7"),

    MENU_AIR_PARAM(pos_est_k_flow, 0.04f, 0.001f, 0.0f, 1.0f, "Estimation"),

    MENU_AIR_PARAM(mode5_img_x_kp, 2.2f,      0.01f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_x_ki, 0.0f,      0.01f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_x_kd, 0.0f,      0.01f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_x_kff, 0.0f,     0.01f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_x_i_limit, 0.0f, 1.0f,  0.0f, 5000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_x_d_lpf, 0.0f,   0.1f,  0.0f,  500.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_y_kp, 2.2f,      0.01f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_y_ki, 0.0f,      0.01f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_y_kd, 0.0f,      0.01f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_y_kff, 0.0f,     0.01f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_y_i_limit, 0.0f, 1.0f,  0.0f, 5000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_img_y_d_lpf, 0.0f,   0.1f,  0.0f,  500.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_x_kp, 0.15f,      0.01f,  0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_x_ki, 0.0f,      0.001f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_x_kd, 0.0f,      0.01f,  0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_x_kff, 0.02f,     0.001f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_x_i_limit, 3.0f, 0.1f,   0.0f, 5000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_x_d_lpf, 0.0f,   0.1f,   0.0f,  500.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_y_kp, 0.15f,      0.01f,  0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_y_ki, 0.0f,      0.001f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_y_kd, 0.0f,      0.01f,  0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_y_kff, 0.02f,     0.001f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_y_i_limit, 3.0f, 0.1f,   0.0f, 5000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_vel_y_d_lpf, 0.0f,   0.1f,   0.0f,  500.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_kp_car_x, 45.0f, 0.1f, 0.0f, 3000.0f, "Mode5"),
    MENU_AIR_PARAM(mode5_kp_car_y, 45.0f, 0.1f, 0.0f, 3000.0f, "Mode5"),

    MENU_AIR_PARAM(mode8_img_x_kp, 2.4f,      0.01f, 0.0f, 3000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_x_ki, 0.0f,      0.01f, 0.0f, 3000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_x_kd, 0.0f,      0.01f, 0.0f, 3000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_x_kff, 0.0f,     0.01f, 0.0f, 3000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_x_i_limit, 0.0f, 1.0f,  0.0f, 5000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_x_d_lpf, 0.0f,   0.1f,  0.0f,  500.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_y_kp, 1.8f,      0.01f, 0.0f, 3000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_y_ki, 0.0f,      0.01f, 0.0f, 3000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_y_kd, 0.0f,      0.01f, 0.0f, 3000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_y_kff, 0.0f,     0.01f, 0.0f, 3000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_y_i_limit, 0.0f, 1.0f,  0.0f, 5000.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_img_y_d_lpf, 0.0f,   0.1f,  0.0f,  500.0f, "Mode8 Img"),
    MENU_AIR_PARAM(mode8_vel_x_kp, 0.15f,      0.01f,  0.0f, 3000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_x_ki, 0.0f,      0.001f, 0.0f, 3000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_x_kd, 0.0f,      0.01f,  0.0f, 3000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_x_kff, 0.0f,     0.001f, 0.0f, 3000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_x_i_limit, 3.0f, 0.1f,   0.0f, 5000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_x_d_lpf, 0.0f,   0.1f,   0.0f,  500.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_y_kp, 0.15f,      0.01f,  0.0f, 3000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_y_ki, 0.0f,      0.001f, 0.0f, 3000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_y_kd, 0.0f,      0.01f,  0.0f, 3000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_y_kff, 0.0f,     0.001f, 0.0f, 3000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_y_i_limit, 3.0f, 0.1f,   0.0f, 5000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_vel_y_d_lpf, 0.0f,   0.1f,   0.0f,  500.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_kp_car_x, 20.0f, 0.1f, 0.0f, 3000.0f, "Mode8 Vel"),
    MENU_AIR_PARAM(mode8_kp_car_y, 20.0f, 0.1f, 0.0f, 3000.0f, "Mode8 Vel")
};

typedef char menu_air_param_count_must_match[
    ((sizeof(s_air_param_definitions) / sizeof(s_air_param_definitions[0])) ==
     MENU_AIR_EXPECTED_PARAM_COUNT) ? 1 : -1];

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

static void menu_air_sync_reset(uint8 mode)
{
    s_air_sync_status.sending = 0U;
    s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;
    s_air_sync_status.mode = mode;
    s_air_sync_next_index = 0U;
}

static void menu_air_mark_catalog_stale(void)
{
    s_air_catalog_ready = 0U;
    s_air_boot_sync_done = 0U;
    s_air_pull_retry_tick = air_comm_car_get_tick();
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

static uint32 menu_air_name_hash(const char *name)
{
    uint32 hash = 2166136261UL;

    if(name == NULL)
    {
        return 0U;
    }

    while(*name != '\0')
    {
        hash ^= (uint8)(*name);
        hash *= 16777619UL;
        name++;
    }

    return hash;
}

static uint32 menu_air_calc_buffer_checksum(uint16 word_count, uint32 offset)
{
    uint16 index;
    uint32 checksum = 0x13572468UL;
    uint32 value_bits;

    for(index = 0U; index < word_count; index++)
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
    uint32 offset;
    uint32 data_words;
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

    offset = (uint32)((sizeof(menu_air_slot_header_t) + 3U) / 4U);
    data_words = (uint32)header->param_count *
                 (uint32)(sizeof(menu_air_slot_param_t) / sizeof(uint32));

    if((header->magic != MENU_AIR_MAGIC_NUMBER) ||
       (header->version != MENU_AIR_VERSION) ||
       (header->slot_id != slot) ||
       (header->param_count == 0U) ||
       (header->param_count > MENU_AIR_MAX_PARAMS) ||
       ((offset + data_words) > FLASH_PAGE_LENGTH))
    {
        return 0U;
    }

    if(header->checksum != menu_air_calc_buffer_checksum((uint16)data_words, offset))
    {
        return 0U;
    }

    if(out_header != NULL)
    {
        *out_header = *header;
    }

    return 1U;
}

/* 从Car Flash读取指定Air存档到本地缓存，不触发串口同步。 */
static uint8 menu_air_load_slot_values(uint8 slot)
{
    uint32 offset;
    uint8 index;
    uint8 load_count;
    float value;
    menu_air_slot_header_t header;
    menu_air_slot_param_t *slot_params;

    if(menu_air_slot_valid(slot, &header) == 0U)
    {
        return 1U;
    }

    for(index = 0U; index < s_air_param_count; index++)
    {
        value = car_math_clampf(s_air_param_definitions[index].default_val,
                                s_air_params[index].min_val,
                                s_air_params[index].max_val);
        *(s_air_params[index].variable) = value;
    }

    offset = (uint32)((sizeof(menu_air_slot_header_t) + 3U) / 4U);
    slot_params = (menu_air_slot_param_t *)&flash_union_buffer[offset];
    load_count = (header.param_count < s_air_param_count) ?
                 (uint8)header.param_count : s_air_param_count;

    for(index = 0U; index < load_count; index++)
    {
        if(slot_params[index].name_hash != menu_air_name_hash(s_air_params[index].name))
        {
            break;
        }

        value = slot_params[index].value;
        value = car_math_clampf(value, s_air_params[index].min_val, s_air_params[index].max_val);
        *(s_air_params[index].variable) = value;
    }

    menu_air_clear_dirty();
    return 0U;
}

void menu_air_support_init(void)
{
    uint8 index;
    const menu_air_param_definition_t *definition;

    s_air_param_count = 0U;
    memset(s_air_param_values, 0, sizeof(s_air_param_values));
    memset(s_air_param_dirty, 0, sizeof(s_air_param_dirty));
    memset(s_air_params, 0, sizeof(s_air_params));
    memset(&s_air_sync_status, 0, sizeof(s_air_sync_status));
    memset(&s_air_cmd_status, 0, sizeof(s_air_cmd_status));
    s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;
    s_air_sync_status.last_failed_index = MENU_AIR_SYNC_INVALID_INDEX;
    s_air_sync_status.mode = MENU_AIR_SYNC_MODE_IDLE;
    s_air_sync_status.reason = MENU_AIR_SYNC_REASON_NONE;
    s_air_sync_next_index = 0U;
    s_air_last_online = 0U;
    s_air_boot_sync_done = 0U;
    s_air_boot_override_done = 0U;
    s_air_catalog_ready = 0U;
    s_air_pull_retry_tick = 0U;
    s_air_deferred_error_reason = MENU_AIR_SYNC_REASON_NONE;
    s_air_cmd_status.state = MENU_AIR_CMD_STATE_IDLE;
    s_air_cmd_status.active_index = MENU_AIR_CMD_INVALID_INDEX;

    for(index = 0U; index < MENU_AIR_EXPECTED_PARAM_COUNT; index++)
    {
        definition = &s_air_param_definitions[index];
        strncpy(s_air_params[index].name,
                definition->name,
                sizeof(s_air_params[index].name) - 1U);
        s_air_params[index].name[sizeof(s_air_params[index].name) - 1U] = '\0';
        s_air_params[index].variable = &s_air_param_values[index];
        s_air_params[index].step = definition->step;
        s_air_params[index].min_val = definition->min_val;
        s_air_params[index].max_val = definition->max_val;
        s_air_params[index].menu_name = definition->menu_name;
        s_air_param_values[index] = car_math_clampf(definition->default_val,
                                                   definition->min_val,
                                                   definition->max_val);
    }
    s_air_param_count = MENU_AIR_EXPECTED_PARAM_COUNT;
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
    if(car_menu_is_runtime_locked() != 0U)
    {
        return 0U;
    }

    if((menu_is_air_connected() == 0U) || (s_air_catalog_ready == 0U))
    {
        return 0U;
    }

    return 1U;
}

uint8 menu_sync_all_air_params(void)
{
    return menu_air_sync_all_start(MENU_AIR_SYNC_REASON_MANUAL);
}

uint8 menu_air_sync_all_start(uint8 reason)
{
    if(car_menu_is_runtime_locked() != 0U)
    {
        return 1U;
    }

    if((menu_can_edit_air_params() == 0U) ||
       (s_air_param_count == 0U) ||
       (s_air_sync_status.sending != 0U) ||
       (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_COMMIT) ||
       (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_FULL) ||
       (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_PULL) ||
       (menu_air_command_is_active() != 0U) ||
       (air_comm_car_has_pending_ack() != 0U))
    {
        if(reason == MENU_AIR_SYNC_REASON_MANUAL)
        {
            if(menu_is_air_connected() == 0U)
            {
                menu_show_error("Air Offline");
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

    return 0U;
}

uint8 menu_air_is_busy(void)
{
    return ((s_air_sync_status.sending != 0U) ||
            (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_COMMIT) ||
            (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_FULL) ||
            (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_PULL) ||
            (menu_air_command_is_active() != 0U) ||
            (air_comm_car_has_pending_ack() != 0U)) ? 1U : 0U;
}

void menu_air_stop_param_sync(void)
{
    uint8 mode = s_air_sync_status.mode;

    if((mode != MENU_AIR_SYNC_MODE_COMMIT) &&
       (mode != MENU_AIR_SYNC_MODE_FULL) &&
       (mode != MENU_AIR_SYNC_MODE_PULL))
    {
        return;
    }

    if((s_air_sync_status.sending != 0U) &&
       (s_air_sync_status.active_index < s_air_param_count))
    {
        menu_air_record_failure(s_air_sync_status.active_index,
                                AIR_COMM_ACK_RESULT_TIMEOUT,
                                AIR_COMM_STATUS_ERROR);
    }

    if((mode == MENU_AIR_SYNC_MODE_COMMIT) ||
       (mode == MENU_AIR_SYNC_MODE_FULL))
    {
        s_air_deferred_error_reason = s_air_sync_status.reason;
    }

    menu_air_sync_reset(MENU_AIR_SYNC_MODE_FAIL);
    menu_air_mark_catalog_stale();
    if(mode == MENU_AIR_SYNC_MODE_PULL)
    {
        air_comm_car_cancel_pending_get_param();
    }
    else
    {
        air_comm_car_cancel_pending_set_param();
    }
}

void menu_air_abort_param_sync_runtime(void)
{
    uint8 mode = s_air_sync_status.mode;

    if((mode != MENU_AIR_SYNC_MODE_COMMIT) &&
       (mode != MENU_AIR_SYNC_MODE_FULL) &&
       (mode != MENU_AIR_SYNC_MODE_PULL))
    {
        return;
    }

    if(mode == MENU_AIR_SYNC_MODE_PULL)
    {
        air_comm_car_cancel_pending_get_param();
    }
    else
    {
        air_comm_car_cancel_pending_set_param();
    }

    s_air_deferred_error_reason = MENU_AIR_SYNC_REASON_NONE;
    menu_air_sync_reset(MENU_AIR_SYNC_MODE_FAIL);
    menu_air_mark_catalog_stale();
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
    if((index >= s_air_param_count) || (s_air_params[index].variable == NULL))
    {
        return 1U;
    }

    if(car_menu_is_runtime_locked() != 0U)
    {
        return 1U;
    }

    if(menu_can_edit_air_params() == 0U)
    {
        menu_show_error((menu_is_air_connected() == 0U) ? "Air Offline" : "Air Not Ready");
        return 1U;
    }

    if(menu_air_is_busy() != 0U)
    {
        menu_show_error("Air Busy");
        return 1U;
    }

    value = car_math_clampf(value,
                            s_air_params[index].min_val,
                            s_air_params[index].max_val);
    menu_show_progress("Syncing");
    air_comm_car_clear_last_ack();
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
    return 0U;
}

static uint8 menu_air_pull_all_start(void)
{
    if((car_menu_is_runtime_locked() != 0U) ||
       (menu_is_air_connected() == 0U) ||
       (s_air_param_count != MENU_AIR_EXPECTED_PARAM_COUNT) ||
       (s_air_sync_status.sending != 0U) ||
       (menu_air_command_is_active() != 0U) ||
       (air_comm_car_has_pending_ack() != 0U))
    {
        return 1U;
    }

    menu_air_sync_reset(MENU_AIR_SYNC_MODE_PULL);
    s_air_sync_status.reason = MENU_AIR_SYNC_REASON_BOOT;
    s_air_sync_status.last_result = AIR_COMM_ACK_RESULT_NONE;
    s_air_sync_status.last_status = AIR_COMM_STATUS_ERROR;
    s_air_catalog_ready = 0U;
    return 0U;
}

void menu_air_update_100HZ(void)
{
    uint8 ack_type = 0U;
    uint8 ack_result = AIR_COMM_ACK_RESULT_NONE;
    uint8 ack_status = AIR_COMM_STATUS_ERROR;
    uint8 active_index;
    uint8 expected_type;
    uint8 online = menu_is_air_connected();
    uint8 current_mode;
    uint8 sync_reason;
    uint32 now = air_comm_car_get_tick();
    char ack_name[AIR_COMM_PARAM_NAME_MAX + 1U];

    if(car_menu_is_runtime_locked() != 0U)
    {
        menu_air_abort_param_sync_runtime();
        return;
    }

    if(s_air_deferred_error_reason != MENU_AIR_SYNC_REASON_NONE)
    {
        sync_reason = s_air_deferred_error_reason;
        s_air_deferred_error_reason = MENU_AIR_SYNC_REASON_NONE;
        if(sync_reason == MENU_AIR_SYNC_REASON_LOAD)
        {
            menu_show_error("Air Load Fail");
        }
        else if(sync_reason == MENU_AIR_SYNC_REASON_MANUAL)
        {
            menu_show_error("Air Sync Fail");
        }
        else if(sync_reason == MENU_AIR_SYNC_REASON_BOOT_OVERRIDE)
        {
            menu_show_error("Air Boot Fail");
        }
        else
        {
            menu_show_error("Air Fail");
        }
        return;
    }

    if(online == 0U)
    {
        if(s_air_last_online != 0U)
        {
            if((s_air_sync_status.mode == MENU_AIR_SYNC_MODE_PULL) ||
               (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_COMMIT) ||
               (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_FULL))
            {
                menu_air_stop_param_sync();
            }
            else
            {
                menu_air_mark_catalog_stale();
            }
        }
        s_air_last_online = 0U;
        return;
    }

    if((s_air_catalog_ready == 0U) &&
       (s_air_boot_sync_done == 0U) &&
       (s_air_sync_status.sending == 0U) &&
       ((s_air_sync_status.mode == MENU_AIR_SYNC_MODE_IDLE) ||
        (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_DONE) ||
        (s_air_sync_status.mode == MENU_AIR_SYNC_MODE_FAIL)) &&
       (menu_air_command_is_active() == 0U) &&
       (air_comm_car_has_pending_ack() == 0U) &&
       ((s_air_last_online == 0U) ||
        ((now - s_air_pull_retry_tick) >= MENU_AIR_PULL_RETRY_MS)))
    {
        if(MENU_AIR_BOOT_OVERRIDE_ENABLE != 0U)
        {
            if(s_air_boot_override_done == 0U)
            {
                if(MENU_AIR_BOOT_FLASH_LOAD_ENABLE != 0U)
                {
                    /* Flash无有效存档时保留Car代码中的Air参数作为覆盖值。 */
                    (void)menu_air_load_slot_values(MENU_AIR_BOOT_FLASH_LOAD_SLOT);
                }
                s_air_catalog_ready = 1U;
                if(menu_air_sync_all_start(MENU_AIR_SYNC_REASON_BOOT_OVERRIDE) != 0U)
                {
                    menu_air_mark_catalog_stale();
                    s_air_pull_retry_tick = now;
                }
            }
        }
        else if(menu_air_pull_all_start() != 0U)
        {
            s_air_pull_retry_tick = now;
        }
    }
    s_air_last_online = 1U;

    if(s_air_sync_status.sending != 0U)
    {
        (void)air_comm_car_get_last_ack(&ack_type, &ack_result, &ack_status);
        if(air_comm_car_has_pending_ack() != 0U)
        {
            return;
        }

        active_index = s_air_sync_status.active_index;
        current_mode = s_air_sync_status.mode;
        expected_type = (current_mode == MENU_AIR_SYNC_MODE_PULL) ?
                        MENU_AIR_ACK_TYPE_GET_PARAM : MENU_AIR_ACK_TYPE_SET_PARAM;
        s_air_sync_status.sending = 0U;
        s_air_sync_status.active_index = MENU_AIR_SYNC_INVALID_INDEX;
        s_air_sync_status.last_result = ack_result;
        s_air_sync_status.last_status = ack_status;

        ack_name[0] = '\0';
        (void)air_comm_car_get_last_ack_name(ack_name, (uint8)sizeof(ack_name));
        if((active_index >= s_air_param_count) ||
           (ack_type != expected_type) ||
           (ack_result == AIR_COMM_ACK_RESULT_NONE) ||
           (strcmp(ack_name, s_air_params[active_index].name) != 0) ||
           ((current_mode == MENU_AIR_SYNC_MODE_PULL) &&
            ((ack_result != AIR_COMM_ACK_RESULT_OK) || (ack_status != AIR_COMM_STATUS_OK))) ||
           ((current_mode != MENU_AIR_SYNC_MODE_PULL) &&
            (menu_air_ack_is_success(ack_result, ack_status) == 0U)))
        {
            menu_air_record_failure(active_index, ack_result, ack_status);
            menu_air_sync_reset(MENU_AIR_SYNC_MODE_FAIL);
            menu_air_mark_catalog_stale();
            if(current_mode == MENU_AIR_SYNC_MODE_PULL)
            {
                return;
            }
            if(current_mode == MENU_AIR_SYNC_MODE_COMMIT)
            {
                menu_show_error((ack_result == AIR_COMM_ACK_RESULT_TIMEOUT) ?
                                "Air Timeout" : "Air Fail");
            }
            else if(current_mode == MENU_AIR_SYNC_MODE_FULL)
            {
                if(s_air_sync_status.reason == MENU_AIR_SYNC_REASON_LOAD)
                {
                    menu_show_error("Air Load Fail");
                }
                else if(s_air_sync_status.reason == MENU_AIR_SYNC_REASON_BOOT_OVERRIDE)
                {
                    menu_show_error("Air Boot Fail");
                }
                else
                {
                    menu_show_error("Air Sync Fail");
                }
            }
            return;
        }

        (void)air_comm_car_get_last_ack_value(s_air_params[active_index].variable);
        s_air_sync_status.ok_count++;
        s_air_param_dirty[active_index] = 0U;

        if(current_mode == MENU_AIR_SYNC_MODE_COMMIT)
        {
            menu_air_sync_reset(MENU_AIR_SYNC_MODE_IDLE);
            menu_show_success("Air OK");
            return;
        }

        if((current_mode == MENU_AIR_SYNC_MODE_FULL) &&
           (s_air_sync_status.dirty_count > 0U))
        {
            s_air_sync_status.dirty_count--;
        }
        return;
    }

    if(s_air_sync_status.mode == MENU_AIR_SYNC_MODE_PULL)
    {
        if(s_air_sync_next_index >= s_air_param_count)
        {
            menu_air_sync_reset(MENU_AIR_SYNC_MODE_DONE);
            menu_air_clear_dirty();
            s_air_catalog_ready = 1U;
            s_air_boot_sync_done = 1U;
            s_air_pull_retry_tick = now;
            menu_request_refresh(REFRESH_FULL);
            return;
        }

        if(air_comm_car_has_pending_ack() != 0U)
        {
            return;
        }

        active_index = s_air_sync_next_index;
        air_comm_car_clear_last_ack();
        if(air_comm_car_get_param(s_air_params[active_index].name) == 0U)
        {
            s_air_sync_status.sending = 1U;
            s_air_sync_status.active_index = active_index;
            s_air_sync_status.send_count++;
            s_air_sync_next_index++;
        }
        else
        {
            menu_air_record_failure(active_index,
                                    AIR_COMM_ACK_RESULT_ERROR,
                                    AIR_COMM_STATUS_ERROR);
            menu_air_sync_reset(MENU_AIR_SYNC_MODE_FAIL);
            menu_air_mark_catalog_stale();
        }
        return;
    }

    if(s_air_sync_status.mode != MENU_AIR_SYNC_MODE_FULL)
    {
        s_air_sync_status.dirty_count = menu_air_dirty_count();
        return;
    }

    if((menu_can_edit_air_params() == 0U) ||
       (air_comm_car_has_pending_ack() != 0U))
    {
        return;
    }

    if(s_air_sync_next_index >= s_air_param_count)
    {
        sync_reason = s_air_sync_status.reason;
        menu_air_sync_reset(MENU_AIR_SYNC_MODE_DONE);
        menu_air_clear_dirty();
        if(sync_reason == MENU_AIR_SYNC_REASON_BOOT_OVERRIDE)
        {
            s_air_boot_override_done = 1U;
            s_air_boot_sync_done = 1U;
            s_air_catalog_ready = 1U;
            menu_show_success("Air Boot OK");
        }
        else
        {
            menu_show_success((sync_reason == MENU_AIR_SYNC_REASON_LOAD) ?
                              "Air Load OK" : "Air Sync OK");
        }
        return;
    }

    active_index = s_air_sync_next_index;
    air_comm_car_clear_last_ack();
    if(air_comm_car_set_param(s_air_params[active_index].name,
                              *(s_air_params[active_index].variable)) == 0U)
    {
        s_air_sync_status.sending = 1U;
        s_air_sync_status.active_index = active_index;
        s_air_sync_status.last_result = AIR_COMM_ACK_RESULT_NONE;
        s_air_sync_status.last_status = AIR_COMM_STATUS_ERROR;
        s_air_sync_status.send_count++;
        s_air_sync_next_index++;
    }
    else
    {
        menu_air_record_failure(active_index,
                                AIR_COMM_ACK_RESULT_ERROR,
                                AIR_COMM_STATUS_ERROR);
        menu_air_sync_reset(MENU_AIR_SYNC_MODE_FAIL);
        menu_air_mark_catalog_stale();
        if(s_air_sync_status.reason == MENU_AIR_SYNC_REASON_LOAD)
        {
            menu_show_error("Air Load Fail");
        }
        else if(s_air_sync_status.reason == MENU_AIR_SYNC_REASON_BOOT_OVERRIDE)
        {
            menu_show_error("Air Boot Fail");
        }
        else
        {
            menu_show_error("Air Sync Fail");
        }
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

uint8 menu_load_air_slot(uint8 slot)
{
    if(car_menu_is_runtime_locked() != 0U)
    {
        return 1U;
    }

    if(menu_can_edit_air_params() == 0U)
    {
        menu_show_error((menu_is_air_connected() == 0U) ? "Air Offline" : "Air Not Ready");
        return 1U;
    }

    if(menu_air_is_busy() != 0U)
    {
        menu_show_error("Air Busy");
        return 1U;
    }

    if(menu_air_load_slot_values(slot) != 0U)
    {
        menu_show_error("No Air Data");
        return 1U;
    }
    if(menu_air_sync_all_start(MENU_AIR_SYNC_REASON_LOAD) != 0U)
    {
        menu_air_mark_catalog_stale();
        menu_show_error("Air Sync Fail");
        return 1U;
    }

    return 0U;
}

uint8 menu_save_air_slot(uint8 slot)
{
    uint32 page;
    uint32 offset;
    uint32 data_words;
    uint8 index;
    menu_air_slot_header_t *header;
    menu_air_slot_param_t *slot_params;

    if(car_menu_is_runtime_locked() != 0U)
    {
        return 1U;
    }

    if(menu_can_edit_air_params() == 0U)
    {
        menu_show_error((menu_is_air_connected() == 0U) ? "Air Offline" : "Air Not Ready");
        return 1U;
    }

    if(menu_air_is_busy() != 0U)
    {
        menu_show_error("Air Busy");
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

    offset = (uint32)((sizeof(menu_air_slot_header_t) + 3U) / 4U);
    data_words = (uint32)s_air_param_count *
                 (uint32)(sizeof(menu_air_slot_param_t) / sizeof(uint32));
    if((offset + data_words) > FLASH_PAGE_LENGTH)
    {
        menu_show_error("Air Data Full");
        return 1U;
    }

    slot_params = (menu_air_slot_param_t *)&flash_union_buffer[offset];
    for(index = 0U; index < s_air_param_count; index++)
    {
        slot_params[index].name_hash = menu_air_name_hash(s_air_params[index].name);
        slot_params[index].value = *(s_air_params[index].variable);
    }
    header->checksum = menu_air_calc_buffer_checksum((uint16)data_words, offset);

    flash_erase_page(0U, page);
    (void)flash_write_page_from_buffer(0U, page, offset + data_words);

    if(menu_air_slot_valid(slot, NULL) == 0U)
    {
        menu_show_error("Air Save Fail");
        return 1U;
    }

    return 0U;
}

static uint8 menu_air_command_ack_text_is(const char *text, const char *token)
{
    size_t token_len;

    if((text == NULL) || (token == NULL))
    {
        return 0U;
    }

    token_len = strlen(token);
    if(strncmp(text, token, token_len) != 0)
    {
        return 0U;
    }

    return ((text[token_len] == '\0') || (text[token_len] == ' ')) ? 1U : 0U;
}

static void menu_air_command_reset(uint8 keep_ack)
{
    s_air_cmd_status.state = MENU_AIR_CMD_STATE_IDLE;
    s_air_cmd_status.active_index = MENU_AIR_CMD_INVALID_INDEX;
    s_air_cmd_status.send_tick = 0U;
    if(keep_ack == 0U)
    {
        s_air_cmd_status.last_result = AIR_COMM_ACK_RESULT_NONE;
        s_air_cmd_status.last_status = AIR_COMM_STATUS_ERROR;
        s_air_cmd_status.last_ack_text[0] = '\0';
    }
    menu_request_refresh(REFRESH_FULL);
}

uint8 menu_air_command_get_count(void)
{
    return 1U;
}

const char *menu_air_command_get_name(uint8 index)
{
    return (index == 0U) ? "beep" : NULL;
}

uint8 menu_air_command_is_active(void)
{
    return (s_air_cmd_status.state != MENU_AIR_CMD_STATE_IDLE) ? 1U : 0U;
}

uint8 menu_air_command_is_running(uint8 index)
{
    return ((index == s_air_cmd_status.active_index) &&
            (s_air_cmd_status.state != MENU_AIR_CMD_STATE_IDLE)) ? 1U : 0U;
}

uint8 menu_air_command_start(uint8 index)
{
    if(car_menu_is_runtime_locked() != 0U)
    {
        return 1U;
    }

    if(index >= menu_air_command_get_count())
    {
        menu_show_error("Command Fail");
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
    if(air_comm_car_exec_command("beep") != 0U)
    {
        menu_show_error("Command Fail");
        return 1U;
    }

    s_air_cmd_status.state = MENU_AIR_CMD_STATE_WAIT_START_ACK;
    s_air_cmd_status.active_index = index;
    s_air_cmd_status.last_result = AIR_COMM_ACK_RESULT_NONE;
    s_air_cmd_status.last_status = AIR_COMM_STATUS_ERROR;
    s_air_cmd_status.last_ack_text[0] = '\0';
    s_air_cmd_status.send_tick = air_comm_car_get_tick();
    menu_request_refresh(REFRESH_FULL);
    return 0U;
}

uint8 menu_air_command_stop(void)
{
    if(menu_air_command_is_active() == 0U)
    {
        return 1U;
    }

    air_comm_car_cancel_pending_command();
    menu_air_command_reset(1U);
    return 0U;
}

void menu_air_command_abort_runtime(void)
{
    if(menu_air_command_is_active() == 0U)
    {
        return;
    }

    air_comm_car_cancel_pending_command();
    menu_air_command_reset(1U);
    air_comm_car_clear_last_ack();
    (void)air_comm_car_exec_command("NONE");
}

void menu_air_command_update_100HZ(void)
{
    uint8 ack_type = 0U;
    uint8 ack_result = AIR_COMM_ACK_RESULT_NONE;
    uint8 ack_status = AIR_COMM_STATUS_ERROR;

    if(car_menu_is_runtime_locked() != 0U)
    {
        if(menu_air_command_is_active() != 0U)
        {
            menu_air_command_abort_runtime();
        }
        return;
    }

    if(s_air_cmd_status.state == MENU_AIR_CMD_STATE_IDLE)
    {
        return;
    }

    if((air_comm_car_get_tick() - s_air_cmd_status.send_tick) >= MENU_AIR_COMMAND_TIMEOUT_MS)
    {
        s_air_cmd_status.timeout_count++;
        s_air_cmd_status.last_result = AIR_COMM_ACK_RESULT_TIMEOUT;
        s_air_cmd_status.last_status = AIR_COMM_STATUS_ERROR;
        strncpy(s_air_cmd_status.last_ack_text,
                "Comm Timeout",
                sizeof(s_air_cmd_status.last_ack_text) - 1U);
        s_air_cmd_status.last_ack_text[sizeof(s_air_cmd_status.last_ack_text) - 1U] = '\0';
        air_comm_car_cancel_pending_command();
        menu_air_command_reset(1U);
        return;
    }

    (void)air_comm_car_get_last_ack(&ack_type, &ack_result, &ack_status);
    if((ack_type != MENU_AIR_ACK_TYPE_COMMAND) ||
       (ack_result == AIR_COMM_ACK_RESULT_NONE))
    {
        return;
    }

    s_air_cmd_status.last_result = ack_result;
    s_air_cmd_status.last_status = ack_status;
    (void)air_comm_car_get_last_command_ack_text(
        s_air_cmd_status.last_ack_text,
        (uint8)sizeof(s_air_cmd_status.last_ack_text));

    if(s_air_cmd_status.state == MENU_AIR_CMD_STATE_WAIT_START_ACK)
    {
        if((ack_result == AIR_COMM_ACK_RESULT_OK) &&
           (ack_status == AIR_COMM_STATUS_OK) &&
           (menu_air_command_ack_text_is(s_air_cmd_status.last_ack_text, "ACK_EXIT_OK") != 0U))
        {
            menu_air_command_reset(1U);
        }
        else if((ack_result == AIR_COMM_ACK_RESULT_OK) &&
                (ack_status == AIR_COMM_STATUS_OK) &&
                (menu_air_command_ack_text_is(s_air_cmd_status.last_ack_text, "ACK_OK") != 0U))
        {
            s_air_cmd_status.state = MENU_AIR_CMD_STATE_INSTANT_RUNNING;
            s_air_cmd_status.send_tick = air_comm_car_get_tick();
            air_comm_car_clear_last_ack();
            menu_request_refresh(REFRESH_FULL);
        }
        else
        {
            menu_air_command_reset(1U);
        }
        return;
    }

    if((menu_air_command_ack_text_is(s_air_cmd_status.last_ack_text, "ACK_EXIT_OK") != 0U) ||
       (ack_result != AIR_COMM_ACK_RESULT_OK))
    {
        menu_air_command_reset(1U);
    }
}

void menu_get_air_command_status(menu_air_cmd_status_t *status)
{
    if(status != NULL)
    {
        *status = s_air_cmd_status;
    }
}
