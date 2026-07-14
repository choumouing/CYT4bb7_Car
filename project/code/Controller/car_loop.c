/*
 * Main loop scheduler.
 *
 * The control tick is split by frequency flags. Heavy work stays in the main
 * loop; PIT interrupt only updates flags.
 */
#include "car_loop.h"

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
static uint16 s_air_comm_beep_tick = 200U;
static uint32 s_beacon_beep_enter_count = 0U;
/* Car yaw rate for Air upload, 10Hz low-pass output, unit: deg/s. */
static float s_car_yaw_rate_lpf_dps = 0.0f;

volatile float g_air_state;
volatile float g_air_crsf_std_ch0;
volatile float g_air_crsf_std_ch1;
volatile float g_air_crsf_std_ch2;
volatile float g_air_crsf_std_ch3;
volatile float g_air_crsf_std_ch4;
volatile float g_air_crsf_std_ch5;
volatile float g_air_crsf_std_ch6;
volatile float g_air_crsf_std_ch7;
volatile float g_air_yaw_angle_target_deg = 0.0f;
volatile float g_air_car_plan_valid = 0.0f;
volatile float g_air_car_plan_strafe_mps = 0.0f;
volatile float g_air_car_plan_forward_mps = 0.0f;
volatile float g_air_beacon_lost_flag = 0.0f;

#define AIR_RUN_DATA_COUNT (14U)

#define AIR_YAW_TARGET_DEG_TO_RAD (-0.017453292519943295f)

static void on_air_data(const float *data, uint8 count)
{
    if (count != AIR_RUN_DATA_COUNT)
    {
        return;
    }

    g_air_crsf_std_ch0 = data[0];
    g_air_crsf_std_ch1 = data[1];
    g_air_crsf_std_ch2 = data[2];
    g_air_crsf_std_ch3 = data[3];
    g_air_crsf_std_ch4 = data[4];
    g_air_crsf_std_ch5 = data[5];
    g_air_crsf_std_ch6 = data[6];
    g_air_crsf_std_ch7 = data[7];
    g_air_state = data[8];
    g_air_yaw_angle_target_deg = data[9];
    g_air_car_plan_valid = data[10];
    g_air_car_plan_strafe_mps = data[11];
    g_air_car_plan_forward_mps = data[12];
    g_air_beacon_lost_flag = data[13];
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
    g_air_car_plan_valid = 0.0f;
    g_air_car_plan_strafe_mps = 0.0f;
    g_air_car_plan_forward_mps = 0.0f;
    g_air_beacon_lost_flag = 0.0f;
    s_car_yaw_rate_lpf_dps = 0.0f;
    s_telemetry_timestamp_count = 0U;
    s_system_time_ms = 0U;
    s_beacon_beep_enter_count = 0U;
    beacon_config_init();
}

void car_loop_init(void)
{
    car_loop_runtime_reset();

    // menu_init();
    // menu_config_init();
    mecanum_motor_init();
    encoder_control_init();
    odometer_init();
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
    s_car_yaw_rate_lpf_dps += 0.06089863f *
                              (g_imufilter_1000hz.gyroz - s_car_yaw_rate_lpf_dps);
    odometer_update_1000HZ();
    beacon_detection_update_1000HZ();
}

static void car_loop_100HZ(void)
{
    float car_data[5];
    float odometer_raw_position[2];
    float odometer_fixed_position[2];
    float beacon_detected_flag;

    s_telemetry_timestamp_count++;
    s_system_time_ms = s_telemetry_timestamp_count * 10U;

    encoder_update_100HZ();
    odometer_update_100HZ();
    beacon_detection_update_100HZ();
    fixator_update_100HZ();

    odometer_raw_position[x] = g_odometer.position[x];
    odometer_raw_position[y] = g_odometer.position[y];
    odometer_fixed_position[x] = g_odometer.position[x];
    odometer_fixed_position[y] = g_odometer.position[y];
    beacon_detected_flag = (g_beacon_detection.enter_event != 0U) ? 1.0f : 0.0f;

    if((g_fixator.pending_fix != 0U) && (g_fixator.last_match_valid != 0U))
    {
        odometer_raw_position[x] = g_fixator.before_position[x];
        odometer_raw_position[y] = g_fixator.before_position[y];
        odometer_fixed_position[x] = g_fixator.fixed_position[x];
        odometer_fixed_position[y] = g_fixator.fixed_position[y];
    }

    if ((g_beacon_detection.enter_event != 0U) &&
        (g_beacon_detection.enter_count != s_beacon_beep_enter_count))
    {
        s_beacon_beep_enter_count = g_beacon_detection.enter_count;
        Beep_Play(100U, 0.10f, 1U);
    }

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


    // 如果车机串口通信离线,车端的蜂鸣器报警,为1s的鸣叫,1s的停止
    if (air_comm_car_is_online() == 0U)
    {
        if (s_air_comm_beep_tick >= 200U)
        {
            s_air_comm_beep_tick = 0U;
            Beep_Enable();
        }
        else if (s_air_comm_beep_tick == 100U)
        {
            Beep_Disable();
        }
        s_air_comm_beep_tick++;
    }
    else if (s_air_comm_beep_tick != 200U)
    {
        s_air_comm_beep_tick = 200U;
        Beep_Disable();
    }

    Beep_Update_100HZ();

    if ((car_control_enabled != 0U) &&
        (car_emergency_stop_active == 0U) &&
        (air_comm_car_is_run_data_fresh() != 0U))
    {
        float yaw_target_rad = 0.0f;

        if(CAR_MODE_4 == car_mode_get())
        {
            yaw_target_rad = car_mode4_get_yaw_target_rad();
        }
        else if((CAR_MODE_2 == car_mode_get()) ||
                (CAR_MODE_5 == car_mode_get()) ||
                (CAR_MODE_8 == car_mode_get()))
        {
            yaw_target_rad = g_air_yaw_angle_target_deg * AIR_YAW_TARGET_DEG_TO_RAD;
        }

        Control_100Hz(car_forward_target, car_strafe_target, yaw_target_rad);
    }
    else
    {
        Control_Stop();
    }

    car_data[0] = g_odometer.body_vel[x]; /* 实际横向速度，右正，m/s */
    car_data[1] = g_odometer.body_vel[y]; /* 实际前向速度，前正，m/s */
    car_data[2] = g_euler.yaw;
    car_data[3] = s_car_yaw_rate_lpf_dps;
    car_data[4] = (float)s_system_time_ms;
    air_comm_send_run_data(car_data, 5);

    // if ((CAR_MODE_5 == car_mode_get()) || (CAR_MODE_8 == car_mode_get()))
    // {
        wifi_justfloat(g_imufilter_1000hz.gyrox,                       /* I1 */
                       g_imufilter_1000hz.gyroy,                       /* I2 */
                       g_imufilter_1000hz.gyroz,                       /* I3 */
                       g_imufilter_1000hz.accx,                        /* I4 */
                       g_imufilter_1000hz.accy,                        /* I5 */
                       g_imufilter_1000hz.accz,                        /* I6 */
                       g_euler.pitch,                                  /* I7 */
                       g_euler.roll,                                   /* I8 */
                       g_euler.yaw,                                    /* I9 */
                       encoder_get_left_front_filtered_count(),         /* I10 */
                       encoder_get_right_front_filtered_count(),        /* I11 */
                       encoder_get_left_rear_filtered_count(),          /* I12 */
                       encoder_get_right_rear_filtered_count(),         /* I13 */
                       g_odometer.body_vel[x],                         /* I14 */
                       g_odometer.body_vel[y],                         /* I15 */
                       g_odometer.vel[x],                              /* I16 */
                       g_odometer.vel[y],                              /* I17 */
                       odometer_raw_position[x],                       /* I18 */
                       odometer_raw_position[y],                       /* I19 */                           /* I20 */
                       odometer_fixed_position[x],                     /* I21 */
                       odometer_fixed_position[y],                     /* I22 */
                       g_air_car_plan_strafe_mps,                      /* I23: Air target velocity X */
                       g_air_car_plan_forward_mps,                     /* I24: Air target velocity Y */
                       g_air_beacon_lost_flag,
                       beacon_detected_flag);                        /* I25 */
    // }



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
    air_comm_car_poll();
}
