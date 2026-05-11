#include "car_loop.h"


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
}

void car_loop_init(void)
{
    car_loop_runtime_reset();

    menu_init();
    menu_config_init();
    mecanum_motor_init();
    encoder_control_init();
    odometer_init();
    camera_spi_init();
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
    pit_init(PIT_CH0, 1000);
}

static void car_loop_1000HZ(void)
{
    IMU_Update_1000HZ();
}

static void car_loop_100HZ(void)
{
    s_telemetry_timestamp_count++;
    s_system_time_ms = s_telemetry_timestamp_count * 10U;

    encoder_update_100HZ();
    odometer_update_100HZ();
    camera_spi_update_100HZ(s_system_time_ms);
    air_comm_car_update_100HZ();
    beacon_detection_update_100HZ();
    menu_update_100HZ();

    if(0U != car_control_enabled)
    {
        control_cascade_speed_loop_update_100HZ(car_forward_target, car_strafe_target);
    }
    else
    {
        control_cascade_stop();
        control_yaw_hold_reset();
    }

    wifi_justfloat_update_100HZ(s_system_time_ms);
}

static void car_loop_50HZ(void)
{
    if(0U != car_control_enabled)
    {
        if(0.0f != car_rotate_target)
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

    if(0U != car_control_enabled)
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
    camera_spi_poll();
    air_comm_car_poll();
}
