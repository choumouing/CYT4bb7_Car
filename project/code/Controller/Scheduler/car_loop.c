#include "car_loop.h"


volatile uint8_t timer_100HZ_flag = 0U;
volatile uint8_t timer_50HZ_flag = 0U;
volatile uint8_t timer_25HZ_flag = 0U;
volatile uint16 g_tick_1000HZ = 0U;

static uint32 s_telemetry_timestamp_count = 0U;
static uint32 s_system_time_ms = 0U;

static void car_loop_runtime_reset(void)
{
    timer_100HZ_flag = 0U;
    timer_50HZ_flag = 0U;
    timer_25HZ_flag = 0U;
    g_tick_1000HZ = 0U;

    s_telemetry_timestamp_count = 0U;
    s_system_time_ms = 0U;
}

void car_loop_init(void)
{
    car_loop_runtime_reset();

    menu_config_init();
    mecanum_motor_init();
    encoder_control_init();
    odometer_init();
    beacon_detection_init();
    IMU_Init_All();
    AccelCalibration_Init();
    IMUCalib_Init();
    control_cascade_init();
    uart_receiver_init();
    wireless_control_init();
    car_mode_init();
    wifi_core_Init();
    ALX_AOA_Init();
    uwb_follow_init();
    target_follow_init();
    target_follow_load_default_targets();
    pit_init(PIT_CH0, 1000);
}

static void car_telemetry_update_100HZ(void)
{
    float imu_acc_x_g = g_imufilter_1000hz.accx;
    float imu_acc_y_g = g_imufilter_1000hz.accy;
    float imu_acc_z_g = g_imufilter_1000hz.accz;
    float imu_gyro_x_dps = g_imufilter_1000hz.gyrox;
    float imu_gyro_y_dps = g_imufilter_1000hz.gyroy;
    float imu_gyro_z_dps = g_imufilter_1000hz.gyroz;

    wifi_justfloat((float)s_system_time_ms,
                   imu_acc_x_g,
                   imu_acc_y_g,
                   imu_acc_z_g,
                   imu_gyro_x_dps,
                   imu_gyro_y_dps,
                   imu_gyro_z_dps,
                   g_euler.roll,
                   g_euler.pitch,
                   g_euler.yaw,
                   g_odometer.strafe_distance,
                   g_odometer.forward_distance,
                   encoder_get_left_front_filtered_count(),
                   encoder_get_right_front_filtered_count(),
                   encoder_get_left_rear_filtered_count(),
                   encoder_get_right_rear_filtered_count());
}

void car_loop_poll(void)
{
    uint16 imu_tick_guard = 0U;

    while((g_tick_1000HZ > 0U) && (imu_tick_guard < 100U))
    {
        g_tick_1000HZ--;
        IMU_Update_1000HZ();
        imu_tick_guard++;
    }

    if(timer_25HZ_flag)
    {
        timer_25HZ_flag = 0U;
        ALX_AOA_Update_25HZ(s_system_time_ms);
        wireless_control_update_25HZ();
        car_mode_update_25HZ(s_system_time_ms);
    }

    if(timer_50HZ_flag)
    {
        timer_50HZ_flag = 0U;
        if((0U != g_car_mode_state.control_enabled) &&
           (0.0f != g_car_mode_state.rotate_target))
        {
            control_yaw_rate_loop_update_50HZ(g_car_mode_state.rotate_target);
        }
        else
        {
            control_yaw_rate_loop_update_50HZ(control_yaw_rate_target);
        }
    }

    if(timer_100HZ_flag)
    {
        timer_100HZ_flag = 0U;
        s_telemetry_timestamp_count++;
        s_system_time_ms = s_telemetry_timestamp_count * 10U;
        encoder_update_100HZ();
        odometer_update_100HZ();
        beacon_detection_update_100HZ();
        if(0U != g_car_mode_state.control_enabled)
        {
            control_cascade_speed_loop_update_100HZ(g_car_mode_state.forward_target,
                                                    g_car_mode_state.strafe_target);
        }
        else
        {
            control_cascade_stop();
        }
        car_telemetry_update_100HZ();
    }

    wifi_core_Poll();
}
