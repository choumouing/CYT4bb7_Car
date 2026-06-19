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
volatile float g_air_yaw_angle_target_deg = 0.0f;
volatile float g_air_sync_time_ms = 0.0f;
volatile float g_air_car_plan_valid = 0.0f;
volatile float g_air_car_plan_strafe_mps = 0.0f;
volatile float g_air_car_plan_forward_mps = 0.0f;
volatile float g_air_car_plan_camera = 0.0f;
volatile float g_air_car_plan_beacon_index = 0.0f;
volatile float g_air_car_plan_dist_px = 0.0f;

#define AIR_RUN_DATA_COUNT (23U)
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
#define AIR_RUN_DATA_YAW_ANGLE_TARGET_DEG (15U)
#define AIR_RUN_DATA_SYNC_TIME_MS (16U)
#define AIR_RUN_DATA_CAR_PLAN_VALID (17U)
#define AIR_RUN_DATA_CAR_PLAN_STRAFE_MPS (18U)
#define AIR_RUN_DATA_CAR_PLAN_FORWARD_MPS (19U)
#define AIR_RUN_DATA_CAR_PLAN_CAMERA (20U)
#define AIR_RUN_DATA_CAR_PLAN_BEACON_INDEX (21U)
#define AIR_RUN_DATA_CAR_PLAN_DIST_PX (22U)

#define AIR_YAW_TARGET_DEG_TO_RAD (-0.017453292519943295f)

static void on_air_data(const float *data, uint8 count)
{
    if (count != AIR_RUN_DATA_COUNT)
    {
        return;
    }

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
    g_air_yaw_angle_target_deg = data[AIR_RUN_DATA_YAW_ANGLE_TARGET_DEG];
    g_air_sync_time_ms = data[AIR_RUN_DATA_SYNC_TIME_MS];
    g_air_car_plan_valid = data[AIR_RUN_DATA_CAR_PLAN_VALID];
    g_air_car_plan_strafe_mps = data[AIR_RUN_DATA_CAR_PLAN_STRAFE_MPS];
    g_air_car_plan_forward_mps = data[AIR_RUN_DATA_CAR_PLAN_FORWARD_MPS];
    g_air_car_plan_camera = data[AIR_RUN_DATA_CAR_PLAN_CAMERA];
    g_air_car_plan_beacon_index = data[AIR_RUN_DATA_CAR_PLAN_BEACON_INDEX];
    g_air_car_plan_dist_px = data[AIR_RUN_DATA_CAR_PLAN_DIST_PX];
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
    g_air_car_plan_camera = 0.0f;
    g_air_car_plan_beacon_index = 0.0f;
    g_air_car_plan_dist_px = 0.0f;
    s_telemetry_timestamp_count = 0U;
    s_system_time_ms = 0U;
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
    beacon_detection_update_1000HZ();
}

static void car_loop_100HZ(void)
{
    float car_data[11];

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

    if ((car_control_enabled != 0U) && (car_emergency_stop_active == 0U))
    {
        float yaw_target_rad = 0.0f;

        if(CAR_MODE_4 == car_mode_get())
        {
            yaw_target_rad = car_mode4_get_yaw_target_rad();
        }
        else if((CAR_MODE_5 == car_mode_get()) || (CAR_MODE_8 == car_mode_get()))
        {
            yaw_target_rad = g_air_yaw_angle_target_deg * AIR_YAW_TARGET_DEG_TO_RAD;
        }

        Control_100Hz(car_forward_target, car_strafe_target, yaw_target_rad);
    }
    else
    {
        Control_Stop();
    }

    if(CAR_MODE_5 == car_mode_get())
    {
        car_data[0] = g_car_mode5_state.velocity_strafe_target_mps;
        car_data[1] = g_car_mode5_state.velocity_forward_target_mps;
    }
    else
    {
        car_data[0] = g_car_mode8_state.velocity_strafe_target_mps;
        car_data[1] = g_car_mode8_state.velocity_forward_target_mps;
    }
    car_data[2] = 0.0f;
    car_data[3] = 0.0f;
    car_data[4] = 0.0f;
    car_data[5] = 0.0f;
    car_data[6] = 0.0f;
    car_data[7] = 0.0f;
    car_data[8] = 0.0f;
    car_data[9] = 0.0f;
    car_data[10] = (float)s_system_time_ms;
    air_comm_send_run_data(car_data, 11);

    if ((CAR_MODE_5 == car_mode_get()) || (CAR_MODE_8 == car_mode_get()))
    {
        wifi_justfloat(g_air_sync_time_ms,                              /* I1 */
                       g_odometer.body_vel[x],                          /* I2 */
                       g_odometer.body_vel[y],                          /* I3 */
                       (CAR_MODE_5 == car_mode_get()) ? g_car_mode5_state.velocity_strafe_target_mps : g_car_mode8_state.velocity_strafe_target_mps,    /* I4 */
                       (CAR_MODE_5 == car_mode_get()) ? g_car_mode5_state.velocity_forward_target_mps : g_car_mode8_state.velocity_forward_target_mps,   /* I5 */
                       g_euler.pitch,                                  /* I6 */
                       g_euler.roll,                                   /* I7 */
                       g_euler.yaw,                                    /* I8 */
                       g_air_yaw_angle_target_deg,                     /* I9 */
                       g_air_car_plan_valid,                           /* I10 */
                       g_air_car_plan_strafe_mps,                      /* I11 */
                       g_air_car_plan_forward_mps,                     /* I12 */
                       g_air_car_plan_camera,                          /* I13 */
                       g_air_car_plan_beacon_index,                    /* I14 */
                       g_air_car_plan_dist_px);                        /* I15 */
    }

    if ( (CAR_MODE_4 == car_mode_get()))
    {
        wifi_justfloat(car_mode4_get_yaw_target_rad(),  /* I1: target yaw */
                       control_yaw_angle_current,       /* I2: measured yaw */
                       control_yaw_rate_target,         /* I3: target yaw rate */
                       control_yaw_rate_current,        /* I4: measured yaw rate */
                       yaw_angle_pid.p_term,            /* I5: yaw angle P */
                       yaw_angle_pid.i_term,            /* I6: yaw angle I */
                       yaw_angle_pid.d_term,            /* I7: yaw angle D */
                       control_yaw_angle_output,        /* I8: yaw angle output */
                       yaw_rate_pid.p_term,             /* I9: yaw rate P */
                       yaw_rate_pid.i_term,             /* I10: yaw rate I */
                       yaw_rate_pid.d_term,             /* I11: yaw rate D */
                       control_yaw_rate_output);        /* I12: yaw rate output */
    }


    // wifi_justfloat((float)car_mode_get(),
    //                g_air_euler_yaw,
    //                g_air_yaw_angle_target_deg,
    //                control_yaw_angle_current,
    //                g_car_mode8_state.velocity_forward_target_mps,
    //                g_car_mode8_state.velocity_strafe_target_mps,
    //                g_car_mode8_state.velocity_forward_feedback_mps,
    //                g_car_mode8_state.velocity_strafe_feedback_mps,
    //                g_car_mode8_state.forward_pid_output,
    //                g_car_mode8_state.strafe_pid_output,
    //                g_car_mode8_state.forward_target,
    //                g_car_mode8_state.strafe_target,
    //                (float)g_car_mode8_state.output_valid);

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
