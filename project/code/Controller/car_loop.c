/*
 * Main loop scheduler.
 *
 * The control tick is split by frequency flags. Heavy work stays in the main
 * loop; PIT interrupt only updates flags.
 */
#include "car_loop.h"
#include "Protocols/CameraSpi/camera_spi.h"

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
volatile float g_air_crsf_std_ch8;

#define AIR_RUN_DATA_BASE_COUNT (7U)
#define AIR_RUN_DATA_CRSF_COUNT (16U)
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
    g_air_crsf_std_ch8 = data[15];
}

static void car_loop_camera_spi_send_100HZ(void)
{
    uint8 id;
    uint8 tx[CAR_IMAGE_SPI_TX_RAW_SIZE];

    for (id = 0U; id < CAR_IMAGE_SPI_BOARD_COUNT; id++)
    {
        memset(tx, 0, sizeof(tx));
        tx[0] = 0x5AU;
        tx[1] = id;
        car_loop_write_u32_le(&tx[2], g_image_spi.tx_counter);

        CameraSpi_SendRaw((camera_spi_slave_id_t)id, tx, (uint16)sizeof(tx));
        g_image_spi.tx_counter++;
    }
}

static void car_loop_camera_spi_clear_targets(uint8 id)
{
    uint8 target_index;

    if (id >= CAR_IMAGE_SPI_BOARD_COUNT)
    {
        return;
    }

    for (target_index = 0U; target_index < CAMERA_SPI_IMAGE_TARGET_COUNT; target_index++)
    {
        g_image_spi.board[id].target[target_index].valid = 0U;
        g_image_spi.board[id].target[target_index].x = 0.0f;
        g_image_spi.board[id].target[target_index].y = 0.0f;
        g_image_spi.board[id].target[target_index].radius = 0.0f;
    }
}

static void car_loop_camera_spi_parse_targets(uint8 id, const uint8 *rx)
{
    uint8 target_index;

    car_loop_camera_spi_clear_targets(id);
    for (target_index = 0U; target_index < CAMERA_SPI_IMAGE_TARGET_COUNT; target_index++)
    {
        const uint8 *slot = &rx[target_index * CAMERA_SPI_IMAGE_TARGET_SLOT_SIZE];

        g_image_spi.board[id].target[target_index].valid = slot[CAMERA_SPI_IMAGE_TARGET_VALID_OFFSET];
        g_image_spi.board[id].target[target_index].x =
            car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_TARGET_X_OFFSET]);
        g_image_spi.board[id].target[target_index].y =
            car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_TARGET_Y_OFFSET]);
        g_image_spi.board[id].target[target_index].radius =
            car_loop_read_float_le(&slot[CAMERA_SPI_IMAGE_TARGET_RADIUS_OFFSET]);
    }
}

static void car_loop_camera_spi_read_100HZ(void)
{
    uint8 id;
    uint16 rx_len;
    uint8 rx[CAR_IMAGE_SPI_RAW_SIZE];

    for (id = 0U; id < CAR_IMAGE_SPI_BOARD_COUNT; id++)
    {
        rx_len = (uint16)sizeof(rx);
        if (CameraSpi_ReceiveRaw((camera_spi_slave_id_t)id, rx, &rx_len) == 0U)
        {
            g_image_spi.board[id].online = 0U;
            g_image_spi.board[id].rx_len = 0U;
            car_loop_camera_spi_clear_targets(id);
            g_image_spi.board[id].miss_count++;
            continue;
        }

        g_image_spi.board[id].online = 1U;
        g_image_spi.board[id].rx_len = (uint8)rx_len;
        g_image_spi.board[id].rx_count++;

        if (rx_len == CAR_IMAGE_SPI_RAW_SIZE)
        {
            car_loop_camera_spi_parse_targets(id, rx);
        }
        else
        {
            car_loop_camera_spi_clear_targets(id);
        }
    }
}

static void car_loop_beacon_fusion_update_100HZ(void)
{
    uint8 board_id;
    uint8 target_index;
    beacon_fusion_camera_frame_t camera[BEACON_FUSION_CAMERA_COUNT];

    memset(camera, 0, sizeof(camera));
    for (board_id = 0U; board_id < BEACON_FUSION_CAMERA_COUNT; board_id++)
    {
        for (target_index = 0U; target_index < BEACON_FUSION_CAMERA_TARGETS; target_index++)
        {
            camera[board_id].target[target_index].valid =
                g_image_spi.board[board_id].target[target_index].valid;
            camera[board_id].target[target_index].x =
                g_image_spi.board[board_id].target[target_index].x;
            camera[board_id].target[target_index].y =
                g_image_spi.board[board_id].target[target_index].y;
            camera[board_id].target[target_index].radius =
                g_image_spi.board[board_id].target[target_index].radius;
        }
    }

    beacon_fusion_update_100HZ(camera);
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

static void car_loop_100HZ(void)
{
    float car_data[10];

    s_telemetry_timestamp_count++;
    s_system_time_ms = s_telemetry_timestamp_count * 10U;

    encoder_update_100HZ();
    odometer_update_100HZ();
    beacon_detection_update_100HZ();

    if (g_beacon_detection.bump_detected != 0U)
    {
        Beep_Play(100, 0.5f, 1);
    }
    else
    {
        Beep_Stop();
    }

    CameraSpi_Poll();
    car_loop_camera_spi_read_100HZ();
    car_loop_beacon_fusion_update_100HZ();
    car_loop_camera_spi_send_100HZ();
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

    car_data[0] = encoder_left_front.count_raw;
    car_data[1] = encoder_right_front.count_raw;
    car_data[2] = encoder_left_rear.count_raw;
    car_data[3] = encoder_right_rear.count_raw;
    car_data[4] = g_imufilter_1000hz.accx;
    car_data[5] = g_imufilter_1000hz.accy;
    car_data[6] = g_imufilter_1000hz.accz;
    car_data[7] = g_imufilter_1000hz.gyrox;
    car_data[8] = g_imufilter_1000hz.gyroy;
    car_data[9] = g_imufilter_1000hz.gyroz;
    air_comm_send_run_data(car_data, 10);



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
    //                g_image_spi.board[0].target[0].radius,
    //                g_image_spi.board[0].target[1].x,
    //                g_image_spi.board[0].target[1].y,
    //                g_image_spi.board[0].target[1].radius,
    //                g_image_spi.board[1].target[0].x,
    //                g_image_spi.board[1].target[0].y,
    //                g_image_spi.board[1].target[0].radius,
    //                g_image_spi.board[1].target[1].x,
    //                g_image_spi.board[1].target[1].y,
    //                g_image_spi.board[1].target[1].radius,
    //                g_image_spi.board[2].target[0].x,
    //                g_image_spi.board[2].target[0].y,
    //                g_image_spi.board[2].target[0].radius,
    //                g_image_spi.board[2].target[1].x,
    //                g_image_spi.board[2].target[1].y,
    //                g_image_spi.board[2].target[1].radius,
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
