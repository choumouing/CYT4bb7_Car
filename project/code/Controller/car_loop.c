/*
 * Main loop scheduler.
 *
 * The control tick is split by frequency flags. Heavy work stays in the main
 * loop; PIT interrupt only updates flags.
 */
#include "car_loop.h"
#include "Protocols/CameraSpi/camera_spi.h"

volatile uint8_t timer_100HZ_flag = 0U;
volatile uint8_t timer_50HZ_flag = 0U;
volatile uint8_t timer_25HZ_flag = 0U;
volatile uint16 g_tick_1000HZ = 0U;

float car_forward_target = 0.0f;
float car_strafe_target = 0.0f;
float car_rotate_target = 0.0f;
uint8 car_control_enabled = 0U;
uint8 car_emergency_stop_active = 1U;

static uint32 s_telemetry_timestamp_count = 0U;
static uint32 s_system_time_ms = 0U;

volatile car_image_spi_state_t g_image_spi;

volatile float g_air_tof1_height_mm;
volatile float g_air_tof2_height_mm;
volatile float g_air_tof3_height_mm;
volatile float g_air_tof4_height_mm;
volatile float g_air_imufilter_1000hz_accx;
volatile float g_air_imufilter_1000hz_accy;
volatile float g_air_imufilter_1000hz_accz;
volatile float g_air_imufilter_1000hz_gyrox;
volatile float g_air_imufilter_1000hz_gyroy;
volatile float g_air_imufilter_1000hz_gyroz;

static uint16 car_loop_read_u16_le(const volatile uint8 *data)
{
    return (uint16)(((uint16)data[1] << 8) | data[0]);
}

static void car_loop_write_u32_le(uint8 *data, uint32 value)
{
    data[0] = (uint8)(value & 0xFFU);
    data[1] = (uint8)((value >> 8) & 0xFFU);
    data[2] = (uint8)((value >> 16) & 0xFFU);
    data[3] = (uint8)((value >> 24) & 0xFFU);
}

static void on_air_data(const float *data, uint8 count)
{
    if(count >= 10U)
    {
        g_air_tof1_height_mm = data[0];
        g_air_tof2_height_mm = data[1];
        g_air_tof3_height_mm = data[2];
        g_air_tof4_height_mm = data[3];
        g_air_imufilter_1000hz_accx  = data[4];
        g_air_imufilter_1000hz_accy  = data[5];
        g_air_imufilter_1000hz_accz  = data[6];
        g_air_imufilter_1000hz_gyrox = data[7];
        g_air_imufilter_1000hz_gyroy = data[8];
        g_air_imufilter_1000hz_gyroz = data[9];
    }
}

static void car_loop_camera_spi_send_100HZ(void)
{
    uint8 id;
    uint8 tx[CAR_IMAGE_SPI_RAW_SIZE];

    for(id = 0U; id < CAR_IMAGE_SPI_BOARD_COUNT; id++)
    {
        memset(tx, 0, sizeof(tx));
        tx[0] = 0x5AU;
        tx[1] = id;
        car_loop_write_u32_le(&tx[2], g_image_spi.tx_counter);

        CameraSpi_SendRaw((camera_spi_slave_id_t)id, tx, (uint16)sizeof(tx));
        g_image_spi.tx_counter++;
    }
}

static void car_loop_camera_spi_read_100HZ(void)
{
    uint8 id;
    uint8 len;
    uint16 rx_len;
    uint8 rx[CAR_IMAGE_SPI_RAW_SIZE];

    for(id = 0U; id < CAR_IMAGE_SPI_BOARD_COUNT; id++)
    {
        rx_len = (uint16)sizeof(rx);
        if(CameraSpi_ReceiveRaw((camera_spi_slave_id_t)id, rx, &rx_len) == 0U)
        {
            g_image_spi.board[id].online = 0U;
            g_image_spi.board[id].miss_count++;
            continue;
        }

        len = (uint8)rx_len;
        if(len > CAR_IMAGE_SPI_RAW_SIZE)
        {
            len = CAR_IMAGE_SPI_RAW_SIZE;
        }

        memset((void *)g_image_spi.board[id].raw, 0, sizeof(g_image_spi.board[id].raw));
        memcpy((void *)g_image_spi.board[id].raw, rx, len);
        g_image_spi.board[id].online = 1U;
        g_image_spi.board[id].rx_len = len;
        g_image_spi.board[id].rx_count++;

        if(len >= 10U)
        {
            g_image_spi.board[id].frame_id = car_loop_read_u16_le(&g_image_spi.board[id].raw[0]);
            g_image_spi.board[id].spot_count = car_loop_read_u16_le(&g_image_spi.board[id].raw[2]);
            g_image_spi.board[id].x = car_loop_read_u16_le(&g_image_spi.board[id].raw[4]);
            g_image_spi.board[id].y = car_loop_read_u16_le(&g_image_spi.board[id].raw[6]);
            g_image_spi.board[id].area = car_loop_read_u16_le(&g_image_spi.board[id].raw[8]);
        }
    }
}

static void car_loop_runtime_reset(void)
{
    timer_100HZ_flag = 0U;
    timer_50HZ_flag = 0U;
    timer_25HZ_flag = 0U;
    g_tick_1000HZ = 0U;

    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
    car_rotate_target = 0.0f;
    car_control_enabled = 0U;
    car_emergency_stop_active = 1U;
    s_telemetry_timestamp_count = 0U;
    s_system_time_ms = 0U;
    memset((void *)&g_image_spi, 0, sizeof(g_image_spi));
}

void car_loop_init(void)
{
    car_loop_runtime_reset();

    menu_init();
    menu_config_init();
    mecanum_motor_init();
    encoder_control_init();
    odometer_init();
    CameraSpi_Init();
    beacon_detection_init();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    control_cascade_init();
    uart_receiver_init();
    sbus_init();
    wireless_control_init();
    car_start_sbus_init();
    car_mode_init();
    wifi_core_Init();
    ALX_AOA_Init();
    air_comm_car_init();
    air_comm_set_run_data_callback(on_air_data);
    pit_init(PIT_CH0, 1000);
}

static void car_loop_1000HZ(void)
{
    IMU_Update_1000HZ();
}

static void car_loop_100HZ(void)
{
    float car_data[10];

    s_telemetry_timestamp_count++;
    s_system_time_ms = s_telemetry_timestamp_count * 10U;

    encoder_update_100HZ();
    odometer_update_100HZ();

    CameraSpi_Poll();
    car_loop_camera_spi_read_100HZ();
    car_loop_camera_spi_send_100HZ();
    CameraSpi_Poll();

    if((car_control_enabled != 0U) && (car_emergency_stop_active == 0U))
    {
        menu_air_stop_param_sync();
    }
    air_comm_car_update_100HZ();
    beacon_detection_update_100HZ();

    if((car_control_enabled == 0U) || (car_emergency_stop_active != 0U))
    {
        menu_air_update_100HZ();
        menu_update_100HZ();
    }
    else
    {
        menu_discard_key_events();
    }

    if(car_control_enabled != 0U)
    {
        control_cascade_speed_loop_update_100HZ(car_forward_target, car_strafe_target);
    }
    else
    {
        control_cascade_stop();
        control_yaw_hold_reset();
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

    wifi_justfloat((float)g_image_spi.board[0].online,
                   (float)g_image_spi.board[0].frame_id,
                   (float)g_image_spi.board[0].x,
                   (float)g_image_spi.board[0].area,
                   (float)g_image_spi.board[0].rx_count,
                   (float)g_image_spi.board[1].online,
                   (float)g_image_spi.board[1].frame_id,
                   (float)g_image_spi.board[1].x,
                   (float)g_image_spi.board[1].area,
                   (float)g_image_spi.board[1].rx_count,
                   (float)g_image_spi.board[2].online,
                   (float)g_image_spi.board[2].frame_id,
                   (float)g_image_spi.board[2].x,
                   (float)g_image_spi.board[2].area,
                   (float)g_image_spi.board[2].rx_count,
                   (float)g_image_spi.tx_counter);
}

static void car_loop_50HZ(void)
{
    if(car_control_enabled != 0U)
    {
        if(car_rotate_target != 0.0f)
        {
            control_yaw_rate_loop_update_50HZ(car_rotate_target);
        }
        else
        {
            control_yaw_rate_loop_update_50HZ(control_yaw_rate_target);
        }
    }
    else
    {
        control_yaw_rate_loop_update_50HZ(0.0f);
    }
}

static void car_loop_25HZ(void)
{
    ALX_AOA_Update_25HZ(s_system_time_ms);
    sbus_update_25HZ();
    wireless_control_update_25HZ();
    car_start_sbus_update_25HZ();
    car_mode_update_25HZ(s_system_time_ms);

    if(car_control_enabled != 0U)
    {
        control_yaw_hold_update_25HZ(car_rotate_target);
    }
    else
    {
        control_yaw_hold_reset();
    }
}

void car_loop_poll(void)
{
    uint16 imu_tick_guard = 0U;

    while((g_tick_1000HZ > 0U) && (imu_tick_guard < 100U))
    {
        g_tick_1000HZ--;
        car_loop_1000HZ();
        imu_tick_guard++;
    }

    if(timer_25HZ_flag)
    {
        timer_25HZ_flag = 0U;
        car_loop_25HZ();
    }

    if(timer_50HZ_flag)
    {
        timer_50HZ_flag = 0U;
        car_loop_50HZ();
    }

    if(timer_100HZ_flag)
    {
        timer_100HZ_flag = 0U;
        car_loop_100HZ();
    }

    wifi_core_Poll();
    CameraSpi_Poll();
    air_comm_car_poll();
}
