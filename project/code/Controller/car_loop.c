/*
 * Main loop scheduler.
 *
 * The control tick is split by frequency flags. Heavy work stays in the main
 * loop; PIT interrupt only updates flags.
 */

#include "car_loop.h"
#include "car_mode.h"
#include "Estimation/Image/beacon_fusion.h"
#include "Protocols/CameraSpi/camera_spi.h"

#define CAR_LOOP_WIFI_TELEMETRY_FLOAT_COUNT (40U)

volatile uint8_t timer_100HZ_flag = 0U;
volatile uint8_t timer_25HZ_flag = 0U;
volatile uint16 g_tick_1000HZ = 0U;
volatile uint32 tick_1000us_cnt = 0U;

float car_forward_target = 0.0f;
float car_strafe_target = 0.0f;
uint8 car_control_enabled = 0U;
uint8 car_emergency_stop_active = 1U;

static uint32 s_telemetry_timestamp_count = 0U;
static uint32 s_system_time_ms = 0U;

volatile car_image_spi_state_t g_image_spi;

volatile float g_air_tof_fused_height_mm;
volatile float g_air_euler_roll;
volatile float g_air_euler_pitch;
volatile float g_air_euler_yaw;
volatile float g_air_pos_est_vel_x;
volatile float g_air_pos_est_vel_y;
volatile float g_air_state;
volatile float g_air_reserved0;
volatile float g_air_reserved1;
volatile float g_air_reserved2;
volatile float g_air_crsf_std_ch0;
volatile float g_air_crsf_std_ch1;
volatile float g_air_crsf_std_ch2;
volatile float g_air_crsf_std_ch3;
volatile float g_air_crsf_std_ch4;
volatile float g_air_crsf_std_ch5;
volatile float g_air_crsf_std_ch6;
volatile float g_air_crsf_std_ch7;

#define AIR_RUN_DATA_BASE_COUNT (7U)
#define AIR_RUN_DATA_CRSF_COUNT (15U)
#define AIR_RUN_DATA_TOF_FUSED_HEIGHT_MM (0U)
#define AIR_RUN_DATA_EULER_ROLL (1U)
#define AIR_RUN_DATA_EULER_PITCH (2U)
#define AIR_RUN_DATA_EULER_YAW (3U)
#define AIR_RUN_DATA_POS_EST_VEL_X (4U)
#define AIR_RUN_DATA_POS_EST_VEL_Y (5U)
#define AIR_RUN_DATA_STATE (6U)
#define AIR_RUN_DATA_CRSF_STD_CH0 (7U)
#define AIR_RUN_DATA_CRSF_STD_CH1 (8U)
#define AIR_RUN_DATA_CRSF_STD_CH2 (9U)
#define AIR_RUN_DATA_CRSF_STD_CH3 (10U)
#define AIR_RUN_DATA_CRSF_STD_CH4 (11U)
#define AIR_RUN_DATA_CRSF_STD_CH5 (12U)
#define AIR_RUN_DATA_CRSF_STD_CH6 (13U)
#define AIR_RUN_DATA_CRSF_STD_CH7 (14U)

static void car_loop_write_u32_le(uint8 *data, uint32 value)
{
    data[0] = (uint8)(value & 0xFFU);
    data[1] = (uint8)((value >> 8) & 0xFFU);
    data[2] = (uint8)((value >> 16) & 0xFFU);
    data[3] = (uint8)((value >> 24) & 0xFFU);
}

static float car_loop_read_float_le(const uint8 *data)
{
    float value;

    memcpy(&value, data, sizeof(value));
    return value;
}

static uint8 car_loop_camera_role_from_board(uint8 board_id)
{
    if (board_id == BEACON_FUSION_CAMERA_FRONT)
    {
        return CAMERA_SPI_IMAGE_BOARD_FRONT;
    }
    if (board_id == BEACON_FUSION_CAMERA_REAR)
    {
        return CAMERA_SPI_IMAGE_BOARD_REAR;
    }

    return CAMERA_SPI_IMAGE_BOARD_CENTER;
}

static void car_loop_clear_camera_board(uint8 board_id)
{
    uint8 target_index;

    g_image_spi.board[board_id].protocol_version = 0U;
    g_image_spi.board[board_id].beacon_count = 0U;
    g_image_spi.board[board_id].car_lamp_count = 0U;
    for (target_index = 0U; target_index < CAMERA_SPI_IMAGE_TARGET_COUNT; target_index++)
    {
        g_image_spi.board[board_id].target[target_index].valid = 0U;
        g_image_spi.board[board_id].target[target_index].x = 0.0f;
        g_image_spi.board[board_id].target[target_index].y = 0.0f;
        g_image_spi.board[board_id].target[target_index].area = 0.0f;
    }
    g_image_spi.board[board_id].car_lamp.valid = 0U;
    g_image_spi.board[board_id].car_lamp.cx = 0.0f;
    g_image_spi.board[board_id].car_lamp.cy = 0.0f;
    g_image_spi.board[board_id].car_lamp.width = 0.0f;
    g_image_spi.board[board_id].car_lamp.length = 0.0f;
    g_image_spi.board[board_id].car_lamp.angle = 0.0f;
}

static uint8 car_loop_parse_camera_payload(uint8 board_id,
                                           const uint8 *rx,
                                           beacon_fusion_camera_frame_t *camera)
{
    uint8 target_index;

    if ((rx == NULL) || (camera == NULL) ||
        (rx[CAMERA_SPI_IMAGE_VERSION_OFFSET] != CAMERA_SPI_IMAGE_PROTOCOL_VERSION))
    {
        return 0U;
    }

    g_image_spi.board[board_id].protocol_version = rx[CAMERA_SPI_IMAGE_VERSION_OFFSET];
    g_image_spi.board[board_id].beacon_count = rx[CAMERA_SPI_IMAGE_BEACON_COUNT_OFFSET];
    g_image_spi.board[board_id].car_lamp_count = rx[CAMERA_SPI_IMAGE_CAR_LAMP_COUNT_OFFSET];

    for (target_index = 0U; target_index < CAMERA_SPI_IMAGE_TARGET_COUNT; target_index++)
    {
        const uint8 *slot = &rx[CAMERA_SPI_IMAGE_BEACON_PACKET_OFFSET +
                               (target_index * CAMERA_SPI_IMAGE_BEACON_SLOT_SIZE)];
        const uint8 valid = slot[CAMERA_SPI_IMAGE_BEACON_VALID_OFFSET];
        const float x = car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_BEACON_X_OFFSET]);
        const float y = car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_BEACON_Y_OFFSET]);
        const float area = car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_BEACON_AREA_OFFSET]);

        g_image_spi.board[board_id].target[target_index].valid = valid;
        g_image_spi.board[board_id].target[target_index].x = x;
        g_image_spi.board[board_id].target[target_index].y = y;
        g_image_spi.board[board_id].target[target_index].area = area;

        camera->target[target_index].valid = valid;
        camera->target[target_index].x = x;
        camera->target[target_index].y = y;
        camera->target[target_index].area = area;
    }

    {
        const uint8 *slot = &rx[CAMERA_SPI_IMAGE_CAR_LAMP_PACKET_OFFSET];

        g_image_spi.board[board_id].car_lamp.valid = slot[CAMERA_SPI_IMAGE_CAR_LAMP_VALID_OFFSET];
        g_image_spi.board[board_id].car_lamp.cx = car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_CAR_LAMP_CX_OFFSET]);
        g_image_spi.board[board_id].car_lamp.cy = car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_CAR_LAMP_CY_OFFSET]);
        g_image_spi.board[board_id].car_lamp.width = car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_CAR_LAMP_WIDTH_OFFSET]);
        g_image_spi.board[board_id].car_lamp.length = car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_CAR_LAMP_LENGTH_OFFSET]);
        g_image_spi.board[board_id].car_lamp.angle = car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_CAR_LAMP_ANGLE_OFFSET]);
    }

    return 1U;
}

static void on_air_data(const float *data, uint8 count)
{
    (void)count;

    g_air_tof_fused_height_mm = data[0];
    g_air_euler_roll = data[1];
    g_air_euler_pitch = data[2];
    g_air_euler_yaw = data[3];
    g_air_pos_est_vel_x = data[4];
    g_air_pos_est_vel_y = data[5];
    g_air_state = data[6];
    g_air_crsf_std_ch0 = data[7];
    g_air_crsf_std_ch1 = data[8];
    g_air_crsf_std_ch2 = data[9];
    g_air_crsf_std_ch3 = data[10];
    g_air_crsf_std_ch4 = data[11];
    g_air_crsf_std_ch5 = data[12];
    g_air_crsf_std_ch6 = data[13];
    g_air_crsf_std_ch7 = data[14];
}

static void car_loop_camera_spi_update_100HZ(void)
{
    uint8 board_id;
    uint16 rx_len;
    uint8 rx[CAR_IMAGE_SPI_RAW_SIZE];
    uint8 tx[CAR_IMAGE_SPI_TX_RAW_SIZE];
    beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT];

    memset(camera, 0, sizeof(camera));
    for (board_id = 0U; board_id < BEACON_FUSION_CAMERA_COUNT; board_id++)
    {
        car_loop_clear_camera_board(board_id);

        rx_len = (uint16)sizeof(rx);
        if (CameraSpi_ReceiveRaw((camera_spi_slave_id_t)board_id, rx, &rx_len) == 0U)
        {
            g_image_spi.board[board_id].online = 0U;
            g_image_spi.board[board_id].rx_len = 0U;
            g_image_spi.board[board_id].miss_count++;
            continue;
        }

        g_image_spi.board[board_id].online = 1U;
        g_image_spi.board[board_id].rx_len = (uint8)rx_len;
        g_image_spi.board[board_id].rx_count++;

        if (rx_len != CAR_IMAGE_SPI_RAW_SIZE)
        {
            continue;
        }

        if (car_loop_parse_camera_payload(board_id, rx, &camera[board_id]) == 0U)
        {
            car_loop_clear_camera_board(board_id);
        }
    }

    beacon_fusion_update_100HZ(camera);

    for (board_id = 0U; board_id < CAR_IMAGE_SPI_BOARD_COUNT; board_id++)
    {
        memset(tx, 0, sizeof(tx));
        tx[0] = 0x5AU;
        tx[1] = car_loop_camera_role_from_board(board_id);
        car_loop_write_u32_le(&tx[2], g_image_spi.tx_counter);
        CameraSpi_SendRaw((camera_spi_slave_id_t)board_id, tx, (uint16)sizeof(tx));
        g_image_spi.tx_counter++;
    }
}

static void car_loop_runtime_reset(void)
{
    timer_100HZ_flag = 0U;
    timer_25HZ_flag = 0U;
    g_tick_1000HZ = 0U;
    tick_1000us_cnt = 0U;

    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
    car_control_enabled = 0U;
    car_emergency_stop_active = 1U;
    s_telemetry_timestamp_count = 0U;
    s_system_time_ms = 0U;
    memset((void *)&g_image_spi, 0, sizeof(g_image_spi));
    beacon_config_init();
    beacon_fusion_init();
}

void car_loop_init(void)
{
    car_loop_runtime_reset();

    // menu_init();
    // menu_config_init();
    mecanum_motor_init();
    encoder_control_init();
    odometer_init();
    CameraSpi_Init();
    beacon_detection_reset();
    fixator_init();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    Control_Init();
    car_mode_init();
    wifi_core_Init();
    ALX_AOA_Init();
    air_comm_car_init();
    air_comm_set_run_data_callback(on_air_data);
    Beep_Init();
    Beep_Play(100, 1.0f, 1);
    pit_init(PIT_CH0, 1000);
}

static void car_loop_1000HZ(void)
{
    IMU_Update_1000HZ();
    beacon_detection_update_1000HZ();
}

static void car_loop_send_wifi_telemetry(void)
{
    float data[CAR_LOOP_WIFI_TELEMETRY_FLOAT_COUNT];

    data[0] = (float)s_system_time_ms;

    data[1] = (float)g_image_spi.board[BEACON_FUSION_CAMERA_FRONT].beacon_count;
    data[2] = (float)g_image_spi.board[BEACON_FUSION_CAMERA_FRONT].target[0].valid;
    data[3] = g_image_spi.board[BEACON_FUSION_CAMERA_FRONT].target[0].x;
    data[4] = g_image_spi.board[BEACON_FUSION_CAMERA_FRONT].target[0].y;
    data[5] = g_image_spi.board[BEACON_FUSION_CAMERA_FRONT].target[0].area;

    data[6] = (float)g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].beacon_count;
    data[7] = (float)g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].target[0].valid;
    data[8] = g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].target[0].x;
    data[9] = g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].target[0].y;
    data[10] = g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].target[0].area;

    data[11] = (float)g_image_spi.board[BEACON_FUSION_CAMERA_REAR].beacon_count;
    data[12] = (float)g_image_spi.board[BEACON_FUSION_CAMERA_REAR].target[0].valid;
    data[13] = g_image_spi.board[BEACON_FUSION_CAMERA_REAR].target[0].x;
    data[14] = g_image_spi.board[BEACON_FUSION_CAMERA_REAR].target[0].y;
    data[15] = g_image_spi.board[BEACON_FUSION_CAMERA_REAR].target[0].area;

    data[16] = (float)g_beacon_fusion.valid;
    data[17] = (float)g_beacon_fusion.center_delta_valid;
    data[18] = (float)g_beacon_fusion.camera_id;
    data[19] = g_beacon_fusion.image_x;
    data[20] = g_beacon_fusion.image_y;
    data[21] = g_beacon_fusion.center_delta_x;
    data[22] = g_beacon_fusion.center_delta_y;
    data[23] = g_beacon_fusion.area;

    data[24] = (float)g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].car_lamp.valid;
    data[25] = g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].car_lamp.cx;
    data[26] = g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].car_lamp.cy;
    data[27] = g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].car_lamp.width;
    data[28] = g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].car_lamp.length;
    data[29] = g_image_spi.board[BEACON_FUSION_CAMERA_CENTER].car_lamp.angle;
    data[30] = (float)g_car_mode2_state.car_position_in_center_window;

    data[31] = (float)g_car_mode2_state.output_valid;
    data[32] = g_car_mode2_state.forward_target;
    data[33] = g_car_mode2_state.strafe_target;
    data[34] = g_car_mode2_state.forward_pid_p_term;
    data[35] = g_car_mode2_state.forward_pid_i_term;
    data[36] = g_car_mode2_state.forward_pid_d_term;
    data[37] = g_car_mode2_state.strafe_pid_p_term;
    data[38] = g_car_mode2_state.strafe_pid_i_term;
    data[39] = g_car_mode2_state.strafe_pid_d_term;

    wifi_justfloat_Array(data, CAR_LOOP_WIFI_TELEMETRY_FLOAT_COUNT);
}

static void car_loop_100HZ(void)
{
    float car_data[10];

    s_telemetry_timestamp_count++;
    s_system_time_ms = s_telemetry_timestamp_count * 10U;

    encoder_update_100HZ();
    odometer_update_100HZ();
    beacon_detection_update_100HZ();
    fixator_update_100HZ();

    if (g_beacon_detection.enter_event != 0U)
    {
        Beep_Enable();
    }
    if (g_beacon_detection.exit_event != 0U)
    {
        Beep_Disable();
    }

    CameraSpi_Poll();
    car_loop_camera_spi_update_100HZ();
    CameraSpi_Poll();

    if ((car_control_enabled != 0U) && (car_emergency_stop_active == 0U))
    {
        // menu_air_stop_param_sync();
    }
    air_comm_car_update_100HZ();

    if ((car_control_enabled == 0U) || (car_emergency_stop_active != 0U))
    {
        // menu_air_update_100HZ();
        // menu_update_100HZ();
    }
    else
    {
        // menu_discard_key_events();
    }

    car_mode_update_100HZ(s_system_time_ms);

    Beep_Update_100HZ();

    if (car_control_enabled != 0U)
    {
        Control_100Hz(car_forward_target, car_strafe_target);
    }
    else
    {
        Control_Stop();
    }

    car_data[0] = g_odometer.vel[x];            //水平横移速度，正值偏右，负值偏左
    car_data[1] = g_odometer.vel[y];            //水平前进速度，正值前进，负值后退
    car_data[2] = g_image_spi.board[1].car_lamp.cx;
    car_data[3] = g_image_spi.board[1].car_lamp.cy;
    car_data[4] = g_image_spi.board[1].car_lamp.width;
    car_data[5] = g_image_spi.board[1].car_lamp.length;
    car_data[6] = g_image_spi.board[1].car_lamp.angle;
    car_data[7] = g_image_spi.board[1].car_lamp.valid;
    car_data[8] = g_image_spi.board[1].target[0].x;
    car_data[9] = g_image_spi.board[1].target[0].y;
    air_comm_send_run_data(car_data, 10);


    car_loop_send_wifi_telemetry();



    // wifi_justfloat(g_air_tof_fused_height_mm,
    //             g_air_euler_roll,
    //             g_air_euler_pitch,
    //             g_air_euler_yaw,
    //             g_air_pos_est_vel_x,
    //             g_air_pos_est_vel_y,
    //             g_air_state,
    //             g_air_crsf_std_ch0,
    //             g_air_crsf_std_ch1,
    //             g_air_crsf_std_ch2,
    //             g_air_crsf_std_ch3,
    //             g_air_crsf_std_ch4,
    //             g_air_crsf_std_ch5,
    //             g_air_crsf_std_ch6,
    //             g_air_crsf_std_ch7);

    // wifi_justfloat(g_image_spi.board[0].target[0].x,
    //                g_image_spi.board[0].target[0].y,
    //                g_image_spi.board[0].target[0].area,
    //                g_image_spi.board[0].target[1].x,
    //                g_image_spi.board[0].target[1].y,
    //                g_image_spi.board[0].target[1].area,
    //                g_image_spi.board[1].target[0].x,
    //                g_image_spi.board[1].target[0].y,
    //                g_image_spi.board[1].target[0].area,
    //                g_image_spi.board[1].target[1].x,
    //                g_image_spi.board[1].target[1].y,
    //                g_image_spi.board[1].target[1].area,
    //                g_image_spi.board[2].target[0].x,
    //                g_image_spi.board[2].target[0].y,
    //                g_image_spi.board[2].target[0].area,
    //                g_image_spi.board[2].target[1].x,
    //                g_image_spi.board[2].target[1].y,
    //                g_image_spi.board[2].target[1].area,
    //                g_beacon_fusion_result.beacon_count,
    //                g_beacon_fusion_result.beacon[0].bearing_deg,
    //                g_beacon_fusion_result.beacon[0].x_body,
    //                g_beacon_fusion_result.beacon[0].y_body,
    //                g_beacon_fusion_result.beacon[0].range_proxy,
    //                g_beacon_fusion_result.beacon[0].confidence,
    //                g_beacon_fusion_result.beacon[1].bearing_deg,
    //                g_beacon_fusion_result.beacon[1].x_body,
    //                g_beacon_fusion_result.beacon[1].y_body,
    //                g_beacon_fusion_result.beacon[1].range_proxy,
    //                g_beacon_fusion_result.beacon[1].confidence);
}

static void car_loop_25HZ(void)
{
    ALX_AOA_Update_25HZ(s_system_time_ms);
    car_mode_update_25HZ(s_system_time_ms);
}

void car_loop_poll(void)
{
    uint16 imu_tick_guard = 0U;

    while ((g_tick_1000HZ > 0U) && (imu_tick_guard < 100U))
    {
        g_tick_1000HZ--;
        car_loop_1000HZ();
        imu_tick_guard++;
    }

    if (timer_25HZ_flag)
    {
        timer_25HZ_flag = 0U;
        car_loop_25HZ();
    }

    if (timer_100HZ_flag)
    {
        timer_100HZ_flag = 0U;
        car_loop_100HZ();
    }

    wifi_core_Poll();
    CameraSpi_Poll();
    air_comm_car_poll();
}
