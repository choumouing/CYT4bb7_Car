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

volatile float g_air_mode2_target_valid = 0.0f;
volatile float g_air_mode2_target_x = 0.0f;
volatile float g_air_mode2_target_y = 0.0f;
volatile float g_air_mode2_car_lamp_valid = 0.0f;
volatile float g_air_mode2_car_lamp_cx = 0.0f;
volatile float g_air_mode2_car_lamp_cy = 0.0f;
volatile float g_air_mode2_lamp_angle_deg = 0.0f;

#define AIR_RUN_DATA_BASE_COUNT (7U)
#define AIR_RUN_DATA_CRSF_COUNT (15U)
#define AIR_RUN_DATA_MODE2_COUNT (22U)
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

static void on_air_data(const float *data, uint8 count)
{
    if (count < AIR_RUN_DATA_CRSF_COUNT)
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

    if(count >= AIR_RUN_DATA_MODE2_COUNT)
    {
        g_air_mode2_target_valid   = data[15];
        g_air_mode2_target_x       = data[16];
        g_air_mode2_target_y       = data[17];
        g_air_mode2_car_lamp_valid = data[18];
        g_air_mode2_car_lamp_cx    = data[19];
        g_air_mode2_car_lamp_cy    = data[20];
        g_air_mode2_lamp_angle_deg = data[21];
    }
    else
    {
        g_air_mode2_target_valid   = 0.0f;
        g_air_mode2_target_x       = 0.0f;
        g_air_mode2_target_y       = 0.0f;
        g_air_mode2_car_lamp_valid = 0.0f;
        g_air_mode2_car_lamp_cx    = 0.0f;
        g_air_mode2_car_lamp_cy    = 0.0f;
        g_air_mode2_lamp_angle_deg = 0.0f;
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
        Control_100Hz(car_forward_target, car_strafe_target);
    }
    else
    {
        Control_Stop();
    }

    car_data[0] = g_odometer.vel[x];            //水平横移速度，正值偏右，负值偏左
    car_data[1] = g_odometer.vel[y];            //水平前进速度，正值前进，负值后退
    car_data[2] = 0.0f;
    car_data[3] = 0.0f;
    car_data[4] = 0.0f;
    car_data[5] = 0.0f;
    car_data[6] = 0.0f;
    car_data[7] = 0.0f;
    car_data[8] = 0.0f;
    car_data[9] = 0.0f;
    air_comm_send_run_data(car_data, 10);

    wifi_justfloat((float)car_mode_get(),
                   g_air_mode2_target_valid,
                   g_air_mode2_target_x,
                   g_air_mode2_target_y,
                   g_car_mode8_state.target_delta_x,
                   g_car_mode8_state.target_delta_y,
                   g_car_mode8_state.target_distance_px,
                   (float)g_car_mode8_state.distance_zone,
                   g_car_mode8_state.target_speed_mps,
                   g_car_mode8_state.forward_command,
                   g_car_mode8_state.strafe_command,
                   (float)g_car_mode8_state.output_valid);

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
