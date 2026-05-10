#include "car_loop.h"


volatile uint8_t timer_10ms_flag = 0U;
volatile uint8_t timer_20ms_flag = 0U;
volatile uint8_t timer_40ms_flag = 0U;
volatile uint16 g_tick_1000HZ = 0U;

static float s_yaw_angle_target = 0.0f;
static float s_remote_forward_target = 0.0f;
static float s_remote_strafe_target = 0.0f;
static float s_remote_rotate_target = 0.0f;
static uint8_t s_remote_last_rotate_active = 0U;
static uint8_t s_yaw_angle_hold_active = 0U;
static uint32 s_telemetry_timestamp_count = 0U;
static uint32 s_system_time_ms = 0U;
static uint8_t s_last_control_mode = WIRELESS_CONTROL_MODE_REMOTE;

static void car_loop_runtime_reset(void);
static void car_imu_update_1khz(void);
static void car_uwb_protocol_update_40ms(void);
static void car_wireless_input_update_40ms(void);
static void car_control_mode_update_40ms(void);
static void car_motion_target_update_40ms(void);
static void car_yaw_hold_update_40ms(void);
static void car_control_update_40ms(void);
static void car_yaw_rate_control_update_20ms(void);
static void car_encoder_estimator_update_10ms(void);
static void car_timebase_update_10ms(void);
static void car_speed_control_update_10ms(void);
static void car_telemetry_update_10ms(void);
static void car_control_update_10ms(void);
static void car_protocol_poll_idle(void);

static void car_loop_runtime_reset(void)
{
    timer_10ms_flag = 0U;
    timer_20ms_flag = 0U;
    timer_40ms_flag = 0U;
    g_tick_1000HZ = 0U;

    s_yaw_angle_target = 0.0f;
    s_remote_forward_target = 0.0f;
    s_remote_strafe_target = 0.0f;
    s_remote_rotate_target = 0.0f;
    s_remote_last_rotate_active = 0U;
    s_yaw_angle_hold_active = 0U;
    s_telemetry_timestamp_count = 0U;
    s_system_time_ms = 0U;
    s_last_control_mode = WIRELESS_CONTROL_MODE_REMOTE;
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
    wifi_core_Init();
    ALX_AOA_Init();
    uwb_follow_init();
    target_follow_init();
    target_follow_load_default_targets();
    pit_init(PIT_CH0, 1000);
}

static void car_imu_update_1khz(void)
{
    IMU_Update_1000HZ();
}

static void car_uwb_protocol_update_40ms(void)
{
    (void)ALX_AOA_Update(s_system_time_ms);
}

static void car_wireless_input_update_40ms(void)
{
    wireless_control_task();
}

static void car_control_mode_update_40ms(void)
{
    if(s_last_control_mode != g_wireless_control_state.mode)
    {
        control_cascade_reset();
        uwb_follow_reset();
        target_follow_reset();
        s_remote_last_rotate_active = 0U;
        s_yaw_angle_hold_active = 0U;
        s_yaw_angle_target = control_get_current_yaw_angle();
        s_last_control_mode = g_wireless_control_state.mode;
    }
}

static void car_motion_target_update_40ms(void)
{
    if(WIRELESS_CONTROL_MODE_UWB_FOLLOW == g_wireless_control_state.mode)
    {
        target_follow_update(s_system_time_ms);
        s_remote_forward_target = (0U != g_target_follow_state.output_valid) ?
                                  g_target_follow_state.forward_target : 0.0f;
        s_remote_strafe_target = (0U != g_target_follow_state.output_valid) ?
                                 g_target_follow_state.strafe_target : 0.0f;
        s_remote_rotate_target = 0.0f;
    }
    else
    {
        s_remote_forward_target = (float)g_wireless_control_state.forward_speed;
        s_remote_strafe_target = (float)g_wireless_control_state.strafe_speed;
        s_remote_rotate_target = (float)g_wireless_control_state.rotate_speed;
    }
}

static void car_yaw_hold_update_40ms(void)
{
    if(0.0f != s_remote_rotate_target)
    {
        s_yaw_angle_target = control_get_current_yaw_angle();
        s_yaw_angle_hold_active = 0U;
    }
    else
    {
        if((0U == s_yaw_angle_hold_active) || (0U != s_remote_last_rotate_active))
        {
            s_yaw_angle_target = control_get_current_yaw_angle();
            control_yaw_angle_loop_reset();
            s_yaw_angle_hold_active = 1U;
        }
        control_yaw_angle_loop_update(s_yaw_angle_target);
    }
    s_remote_last_rotate_active = (0.0f != s_remote_rotate_target) ? 1U : 0U;
}

static void car_control_update_40ms(void)
{
    car_uwb_protocol_update_40ms();
    car_wireless_input_update_40ms();
    car_control_mode_update_40ms();

    if(0U != g_wireless_control_state.control_enabled)
    {
        car_motion_target_update_40ms();
        car_yaw_hold_update_40ms();
    }
    else
    {
        s_remote_forward_target = 0.0f;
        s_remote_strafe_target = 0.0f;
        s_remote_rotate_target = 0.0f;
        s_remote_last_rotate_active = 0U;
        s_yaw_angle_target = control_get_current_yaw_angle();
        s_yaw_angle_hold_active = 0U;
        uwb_follow_reset();
        target_follow_reset();
    }
}

static void car_yaw_rate_control_update_20ms(void)
{
    if((0U != g_wireless_control_state.control_enabled) && (0.0f != s_remote_rotate_target))
    {
        control_yaw_rate_loop_update(s_remote_rotate_target);
    }
    else
    {
        control_yaw_rate_loop_update(control_yaw_rate_target);
    }
}

static void car_encoder_estimator_update_10ms(void)
{
    encoder_update();
    odometer_update();
    beacon_detection_update();
}

static void car_timebase_update_10ms(void)
{
    s_telemetry_timestamp_count++;
    s_system_time_ms = s_telemetry_timestamp_count * 10U;
}

static void car_speed_control_update_10ms(void)
{
    if(0U != g_wireless_control_state.control_enabled)
    {
        control_cascade_speed_loop_update(s_remote_forward_target, s_remote_strafe_target);
    }
    else
    {
        control_cascade_stop();
    }
}

static void car_telemetry_update_10ms(void)
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

static void car_control_update_10ms(void)
{
    car_encoder_estimator_update_10ms();
    car_timebase_update_10ms();
    car_speed_control_update_10ms();
    car_telemetry_update_10ms();
}

static void car_protocol_poll_idle(void)
{
    wifi_core_Poll();
}

void car_loop_poll(void)
{
    uint16 imu_tick_guard = 0U;

    while((g_tick_1000HZ > 0U) && (imu_tick_guard < 100U))
    {
        g_tick_1000HZ--;
        car_imu_update_1khz();
        imu_tick_guard++;
    }

    if(timer_40ms_flag)
    {
        timer_40ms_flag = 0U;
        car_control_update_40ms();
    }

    if(timer_20ms_flag)
    {
        timer_20ms_flag = 0U;
        car_yaw_rate_control_update_20ms();
    }

    if(timer_10ms_flag)
    {
        timer_10ms_flag = 0U;
        car_control_update_10ms();
    }

    car_protocol_poll_idle();
}
