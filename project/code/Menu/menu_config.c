/*********************************************************************************************************************
* 菜单用户配置实现文件 - 参数集中管理
*
* 功能：注册四电机速度环参数，并保留Flash存档读取/保存入口
********************************************************************************************************************/

#include "menu_config.h"

//====================================================参数变量区====================================================
// 轮速PID参数（四个电机共用）
float wheel_kp = 2.3f;                  // 比例系数
float wheel_ki = 0.08f;                  // 积分系数
float wheel_kd = 0.0f;                  // 微分系数
float wheel_output_limit = 5000.0f;     // 输出限幅 (PWM)
float wheel_i_limit = 2500.0f;          // 积分限幅

//====================================================用户函数声明====================================================
float yaw_angle_kp = 2.10f;
float yaw_angle_ki = 0.0f;
float yaw_angle_kd = 3.20f;
float yaw_angle_i_limit = 4.0f;
float yaw_angle_output_limit = 1.95f;

float yaw_rate_kp = 90.0f;
float yaw_rate_ki = 0.8f;
float yaw_rate_kd = 0.0f;
float yaw_rate_i_limit = 120.0f;
float yaw_rate_output_limit = 1000.0f;

float mode1_velocity_smooth_tau_s = 0.12f;
float mode1_velocity_output_limit = 650.0f;
float mode1_velocity_pid_output_limit = 250.0f;
float mode1_velocity_i_limit = 0.0f;
float mode1_velocity_strafe_kp = 80.0f;
float mode1_velocity_strafe_ki = 0.0f;
float mode1_velocity_strafe_kd = 20.0f;
float mode1_velocity_forward_kp = 80.0f;
float mode1_velocity_forward_ki = 0.0f;
float mode1_velocity_forward_kd = 20.0f;

float mode2_image_target_deadband_px = 0.0f;
float mode2_distance_mid_threshold_px = 30.0f;
float mode2_distance_far_threshold_px = 60.0f;
float mode2_distance_near_speed_mps = 1.0f;
float mode2_distance_mid_speed_mps = 1.0f;
float mode2_distance_far_speed_mps = 1.0f;
float mode2_image_pid_output_limit = 300.0f;
float mode2_image_i_limit = 0.0f;
float mode2_image_strafe_kp = 1.0f;
float mode2_image_strafe_ki = 0.0f;
float mode2_image_strafe_kd = 0.0f;
float mode2_image_forward_kp = 0.8f;
float mode2_image_forward_ki = 0.0f;
float mode2_image_forward_kd = 0.0f;
float mode2_max_accel_mps2 = 30.0f;

float s_curve_max_iter = 50.0f;
float s_curve_conv_tol = 0.001f;
float s_curve_min_dist = 5.0f;

static void load_slot_0_function(void);
static void load_slot_1_function(void);
static void save_slot_0_function(void);
static void save_slot_1_function(void);
static void load_air_slot_0_function(void);
static void load_air_slot_1_function(void);
static void save_air_slot_0_function(void);
static void save_air_slot_1_function(void);
static void sync_air_function(void);
static void diag_imu_function(void);
static void diag_encoder_function(void);
static void diag_position_function(void);
static void diag_pid_function(void);
static void diag_air_function(void);

//====================================================菜单树定义====================================================
// 轮速PID子菜单（增量式）
static menu_item_t wheel_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 0},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 1},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 2},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 3},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 4},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t yaw_rate_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 5},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 6},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 7},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 8},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 9},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 加载存档子菜单
static menu_item_t load_slot_menu[] = {
    {"Load Slot0", MENU_TYPE_FUNCTION, .function = load_slot_0_function},
    {"Load Slot1", MENU_TYPE_FUNCTION, .function = load_slot_1_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 保存存档子菜单
static menu_item_t save_slot_menu[] = {
    {"Save Slot0", MENU_TYPE_FUNCTION, .function = save_slot_0_function},
    {"Save Slot1", MENU_TYPE_FUNCTION, .function = save_slot_1_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 主菜单
static menu_item_t yaw_angle_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 10},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 11},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 12},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 13},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 14},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode1_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 15},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 16},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 17},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 18},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 19},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 20},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 21},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 22},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 23},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 24},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode2_image_segment_menu[] = {
    {"Deadband", MENU_TYPE_PARAMETER, .param_index = 25},
    {"DMidPx", MENU_TYPE_PARAMETER, .param_index = 26},
    {"DFarPx", MENU_TYPE_PARAMETER, .param_index = 27},
    {"DNear", MENU_TYPE_PARAMETER, .param_index = 28},
    {"DMid", MENU_TYPE_PARAMETER, .param_index = 29},
    {"DFar", MENU_TYPE_PARAMETER, .param_index = 30},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 31},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 32},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 33},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 34},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 35},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 36},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 37},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 38},
    {"Accel", MENU_TYPE_PARAMETER, .param_index = 39},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_basic_menu[] = {
    {"gyro_dt", MENU_TYPE_AIR_PARAMETER, .param_index = 0},
    {"angle_dt", MENU_TYPE_AIR_PARAMETER, .param_index = 1},
    {"pos_xy_dt", MENU_TYPE_AIR_PARAMETER, .param_index = 2},
    {"pos_z_dt", MENU_TYPE_AIR_PARAMETER, .param_index = 3},
    {"vel_xy_dt", MENU_TYPE_AIR_PARAMETER, .param_index = 4},
    {"vel_z_dt", MENU_TYPE_AIR_PARAMETER, .param_index = 5},
    {"base_throttle", MENU_TYPE_AIR_PARAMETER, .param_index = 6},
    {"roll_mech_trim_deg", MENU_TYPE_AIR_PARAMETER, .param_index = 7},
    {"pitch_mech_trim_deg", MENU_TYPE_AIR_PARAMETER, .param_index = 8},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_gyro_menu[] = {
    {"roll_gyro_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 9},
    {"roll_gyro_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 10},
    {"roll_gyro_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 11},
    {"roll_gyro_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 12},
    {"roll_gyro_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 13},
    {"roll_gyro_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 14},
    {"pitch_gyro_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 15},
    {"pitch_gyro_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 16},
    {"pitch_gyro_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 17},
    {"pitch_gyro_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 18},
    {"pitch_gyro_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 19},
    {"pitch_gyro_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 20},
    {"yaw_gyro_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 21},
    {"yaw_gyro_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 22},
    {"yaw_gyro_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 23},
    {"yaw_gyro_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 24},
    {"yaw_gyro_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 25},
    {"yaw_gyro_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 26},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_angle_menu[] = {
    {"roll_angle_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 27},
    {"roll_angle_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 28},
    {"roll_angle_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 29},
    {"roll_angle_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 30},
    {"roll_angle_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 31},
    {"roll_angle_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 32},
    {"pitch_angle_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 33},
    {"pitch_angle_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 34},
    {"pitch_angle_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 35},
    {"pitch_angle_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 36},
    {"pitch_angle_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 37},
    {"pitch_angle_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 38},
    {"yaw_angle_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 39},
    {"yaw_angle_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 40},
    {"yaw_angle_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 41},
    {"yaw_angle_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 42},
    {"yaw_angle_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 43},
    {"yaw_angle_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 44},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_pos_menu[] = {
    {"pos_x_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 45},
    {"pos_x_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 46},
    {"pos_x_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 47},
    {"pos_x_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 48},
    {"pos_x_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 49},
    {"pos_x_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 50},
    {"pos_y_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 51},
    {"pos_y_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 52},
    {"pos_y_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 53},
    {"pos_y_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 54},
    {"pos_y_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 55},
    {"pos_y_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 56},
    {"pos_z_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 57},
    {"pos_z_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 58},
    {"pos_z_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 59},
    {"pos_z_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 60},
    {"pos_z_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 61},
    {"pos_z_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 62},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_vel_menu[] = {
    {"vel_x_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 63},
    {"vel_x_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 64},
    {"vel_x_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 65},
    {"vel_x_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 66},
    {"vel_x_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 67},
    {"vel_x_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 68},
    {"vel_y_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 69},
    {"vel_y_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 70},
    {"vel_y_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 71},
    {"vel_y_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 72},
    {"vel_y_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 73},
    {"vel_y_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 74},
    {"vel_z_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 75},
    {"vel_z_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 76},
    {"vel_z_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 77},
    {"vel_z_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 78},
    {"vel_z_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 79},
    {"vel_z_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 80},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_mode_menu[] = {
    {"mode1_track_ff_deg_per_cmps", MENU_TYPE_AIR_PARAMETER, .param_index = 81},
    {"mode1_brake_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 82},
    {"mode1_brake_exit_vel_cmps", MENU_TYPE_AIR_PARAMETER, .param_index = 83},
    {"pos_est_k_flow", MENU_TYPE_AIR_PARAMETER, .param_index = 84},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_mode8_menu[] = {
    {"mode8_img_x_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 85},
    {"mode8_img_x_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 86},
    {"mode8_img_x_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 87},
    {"mode8_img_x_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 88},
    {"mode8_img_x_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 89},
    {"mode8_img_x_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 90},
    {"mode8_img_y_kp", MENU_TYPE_AIR_PARAMETER, .param_index = 91},
    {"mode8_img_y_ki", MENU_TYPE_AIR_PARAMETER, .param_index = 92},
    {"mode8_img_y_kd", MENU_TYPE_AIR_PARAMETER, .param_index = 93},
    {"mode8_img_y_kff", MENU_TYPE_AIR_PARAMETER, .param_index = 94},
    {"mode8_img_y_i_limit", MENU_TYPE_AIR_PARAMETER, .param_index = 95},
    {"mode8_img_y_d_lpf", MENU_TYPE_AIR_PARAMETER, .param_index = 96},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t load_air_slot_menu[] = {
    {"Load Air0", MENU_TYPE_FUNCTION, .function = load_air_slot_0_function},
    {"Load Air1", MENU_TYPE_FUNCTION, .function = load_air_slot_1_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t save_air_slot_menu[] = {
    {"Save Air0", MENU_TYPE_FUNCTION, .function = save_air_slot_0_function},
    {"Save Air1", MENU_TYPE_FUNCTION, .function = save_air_slot_1_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_menu[] = {
    {"Basic", MENU_TYPE_SUBMENU, .submenu = air_basic_menu},
    {"Gyro PID", MENU_TYPE_SUBMENU, .submenu = air_gyro_menu},
    {"Angle PID", MENU_TYPE_SUBMENU, .submenu = air_angle_menu},
    {"Pos PID", MENU_TYPE_SUBMENU, .submenu = air_pos_menu},
    {"Vel PID", MENU_TYPE_SUBMENU, .submenu = air_vel_menu},
    {"Mode1/Est", MENU_TYPE_SUBMENU, .submenu = air_mode_menu},
    {"Mode8", MENU_TYPE_SUBMENU, .submenu = air_mode8_menu},
    {"Sync Air", MENU_TYPE_FUNCTION, .function = sync_air_function},
    {"Load Air", MENU_TYPE_SUBMENU, .submenu = load_air_slot_menu},
    {"Save Air", MENU_TYPE_SUBMENU, .submenu = save_air_slot_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t diag_menu[] = {
    {"IMU", MENU_TYPE_DIAG_VIEW, .function = diag_imu_function},
    {"Encoder", MENU_TYPE_DIAG_VIEW, .function = diag_encoder_function},
    {"Position", MENU_TYPE_DIAG_VIEW, .function = diag_position_function},
    {"PID", MENU_TYPE_DIAG_VIEW, .function = diag_pid_function},
    {"Air Ack", MENU_TYPE_DIAG_VIEW, .function = diag_air_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t main_menu[] = {
    {"Wheel PID", MENU_TYPE_SUBMENU, .submenu = wheel_pid_menu},
    {"YawRate PID", MENU_TYPE_SUBMENU, .submenu = yaw_rate_pid_menu},
    {"YawAng PID", MENU_TYPE_SUBMENU, .submenu = yaw_angle_pid_menu},
    {"Mode1 Vel", MENU_TYPE_SUBMENU, .submenu = mode1_velocity_pid_menu},
    {"Mode2 Img", MENU_TYPE_SUBMENU, .submenu = mode2_image_segment_menu},
    {"Air", MENU_TYPE_SUBMENU, .submenu = air_menu},
    {"Diag", MENU_TYPE_SUBMENU, .submenu = diag_menu},
    {"Load Slot", MENU_TYPE_SUBMENU, .submenu = load_slot_menu},
    {"Save Slot", MENU_TYPE_SUBMENU, .submenu = save_slot_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

//====================================================用户配置初始化====================================================
void menu_config_init(void)
{
    // 注册轮速PID参数（四个电机共用）
    menu_register_param(&wheel_kp, 0.1f, 0.0f, 100.0f);                    // 参数0
    menu_register_param(&wheel_ki, 0.1f, 0.0f, 100.0f);                    // 参数1
    menu_register_param(&wheel_kd, 0.1f, 0.0f, 100.0f);                    // 参数2
    menu_register_param(&wheel_output_limit, 100.0f, 1000.0f, 10000.0f);   // 参数3
    menu_register_param(&wheel_i_limit, 100.0f, 0.0f, 10000.0f);           // 参数4 积分限幅

    menu_register_param(&yaw_rate_kp, 0.1f, 0.0f, 500.0f);
    menu_register_param(&yaw_rate_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&yaw_rate_kd, 0.1f, 0.0f, 500.0f);
    menu_register_param(&yaw_rate_i_limit, 0.1f, 0.0f, 1000.0f);
    menu_register_param(&yaw_rate_output_limit, 1.0f, 0.0f, 5000.0f);

    menu_register_param(&yaw_angle_kp, 0.1f, 0.0f, 50.0f);
    menu_register_param(&yaw_angle_ki, 0.01f, 0.0f, 50.0f);
    menu_register_param(&yaw_angle_kd, 0.01f, 0.0f, 50.0f);
    menu_register_param(&yaw_angle_i_limit, 0.1f, 0.0f, 100.0f);
    menu_register_param(&yaw_angle_output_limit, 0.1f, 0.0f, 10.0f);

    menu_register_param(&mode1_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode1_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode1_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode1_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode1_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode1_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode1_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode1_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode1_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode1_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&mode2_image_target_deadband_px, 0.5f, 0.0f, 50.0f);
    menu_register_param(&mode2_distance_mid_threshold_px, 1.0f, 0.0f, 220.0f);
    menu_register_param(&mode2_distance_far_threshold_px, 1.0f, 0.0f, 220.0f);
    menu_register_param(&mode2_distance_near_speed_mps, 0.01f, 0.0f, 2.0f);
    menu_register_param(&mode2_distance_mid_speed_mps, 0.01f, 0.0f, 2.0f);
    menu_register_param(&mode2_distance_far_speed_mps, 0.01f, 0.0f, 2.0f);
    menu_register_param(&mode2_image_pid_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode2_image_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode2_image_strafe_kp, 0.1f, 0.0f, 500.0f);
    menu_register_param(&mode2_image_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode2_image_strafe_kd, 0.1f, 0.0f, 500.0f);
    menu_register_param(&mode2_image_forward_kp, 0.1f, 0.0f, 500.0f);
    menu_register_param(&mode2_image_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode2_image_forward_kd, 0.1f, 0.0f, 500.0f);
    menu_register_param(&mode2_max_accel_mps2, 0.05f, 0.0f, 5.0f);

    menu_air_support_init();
    menu_set_root(main_menu);
}

//====================================================用户函数实现====================================================
static void load_slot_0_function(void)
{
    menu_load_slot(0);
}

static void load_slot_1_function(void)
{
    menu_load_slot(1);
}

static void save_slot_0_function(void)
{
    menu_save_slot(0);
}

static void save_slot_1_function(void)
{
    menu_save_slot(1);
}

static void load_air_slot_0_function(void)
{
    if(menu_load_air_slot(0U) == 0U)
    {
        menu_show_success("Air Load OK");
    }
}

static void load_air_slot_1_function(void)
{
    if(menu_load_air_slot(1U) == 0U)
    {
        menu_show_success("Air Load OK");
    }
}

static void save_air_slot_0_function(void)
{
    if(menu_save_air_slot(0U) == 0U)
    {
        menu_show_success("Air Save OK");
    }
}

static void save_air_slot_1_function(void)
{
    if(menu_save_air_slot(1U) == 0U)
    {
        menu_show_success("Air Save OK");
    }
}

static void sync_air_function(void)
{
    if(menu_sync_all_air_params() == 0U)
    {
        menu_show_progress("Air Syncing");
    }
}

static const char *diag_air_sync_mode_text(uint8 mode)
{
    switch(mode)
    {
        case MENU_AIR_SYNC_MODE_COMMIT:
            return "commit";

        case MENU_AIR_SYNC_MODE_FULL:
            return "syncing";

        case MENU_AIR_SYNC_MODE_DONE:
            return "done";

        case MENU_AIR_SYNC_MODE_FAIL:
            return "fail";

        default:
            return "idle";
    }
}

/* 显示一行诊断文本（line 0-7，每行16像素高） */
static void diag_show_line(uint8 line, const char *text)
{
    if(line >= MENU_MAX_VISIBLE_LINES)
    {
        return;
    }

    ips114_show_string(0, (uint16)(line * 16U), text);
}

/* 诊断页初始化：清屏 + 设置默认颜色字体 */
static void diag_begin(void)
{
    ips114_clear();
    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_set_font(UI_FONT_NORMAL);
}

/* 诊断页：IMU数据（欧拉角 + 陀螺仪 + 加速度计） */
static void diag_imu_function(void)
{
    char text[32];

    diag_begin();
    sprintf(text, "IMU RPY");
    diag_show_line(0U, text);
    sprintf(text, "R:%7.2f", (double)g_euler.roll);
    diag_show_line(1U, text);
    sprintf(text, "P:%7.2f Y:%7.2f", (double)g_euler.pitch, (double)g_euler.yaw);
    diag_show_line(2U, text);
    sprintf(text, "Gx:%7.2f", (double)g_imufilter_1000hz.gyrox);
    diag_show_line(3U, text);
    sprintf(text, "Gy:%7.2f Gz:%7.2f", (double)g_imufilter_1000hz.gyroy, (double)g_imufilter_1000hz.gyroz);
    diag_show_line(4U, text);
    sprintf(text, "Ax:%6.3f Ay:%6.3f", (double)g_imufilter_1000hz.accx, (double)g_imufilter_1000hz.accy);
    diag_show_line(5U, text);
    sprintf(text, "Az:%6.3f Ready:%u", (double)g_imufilter_1000hz.accz, (unsigned int)g_imu_ready);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* 诊断页：四轮编码器（滤波值 + 原始值） */
static void diag_encoder_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Encoder filt");
    sprintf(text, "LF:%7.1f", (double)encoder_get_left_front_filtered_count());
    diag_show_line(1U, text);
    sprintf(text, "RF:%7.1f", (double)encoder_get_right_front_filtered_count());
    diag_show_line(2U, text);
    sprintf(text, "LR:%7.1f", (double)encoder_get_left_rear_filtered_count());
    diag_show_line(3U, text);
    sprintf(text, "RR:%7.1f", (double)encoder_get_right_rear_filtered_count());
    diag_show_line(4U, text);
    sprintf(text, "Raw %d %d", (int)encoder_get_left_front_count(), (int)encoder_get_right_front_count());
    diag_show_line(5U, text);
    sprintf(text, "Raw %d %d", (int)encoder_get_left_rear_count(), (int)encoder_get_right_rear_count());
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* 诊断页：位置信息（里程计 + UWB原始/滤波坐标） */
static void diag_position_function(void)
{
    char text[32];
    ALX_AOA_Position_t uwb = {0};
    float filt_x_cm = 0.0f;
    float filt_y_cm = 0.0f;
    uint8 uwb_ok;

    uwb_ok = ALX_AOA_GetLatest(&uwb);
    (void)ALX_AOA_GetFilteredXY(&filt_x_cm, &filt_y_cm);

    diag_begin();
    sprintf(text, "Odo Xr:%6.3f", (double)g_odometer.position[x]);
    diag_show_line(0U, text);
    sprintf(text, "Odo Yf:%6.3f", (double)g_odometer.position[y]);
    diag_show_line(1U, text);
    sprintf(text, "Vel R/F:%4.1f%4.1f",
            (double)g_odometer.vel[x],
            (double)g_odometer.vel[y]);
    diag_show_line(2U, text);
    sprintf(text, "UWB ok:%u", (unsigned int)uwb_ok);
    diag_show_line(3U, text);
    sprintf(text, "Raw X:%ld", (long)uwb.x_cm);
    diag_show_line(4U, text);
    sprintf(text, "Raw Y:%ld", (long)uwb.y_cm);
    diag_show_line(5U, text);
    sprintf(text, "Filt:%5.1f %5.1f", (double)filt_x_cm, (double)filt_y_cm);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* 诊断页：PID中间变量（航向环 + 左前轮P/I/Output） */
static void diag_pid_function(void)
{
    char text[32];

    diag_begin();
    sprintf(text, "YawCur:%6.3f", (double)control_yaw_angle_current);
    diag_show_line(0U, text);
    sprintf(text, "YawOut:%6.3f", (double)control_yaw_angle_output);
    diag_show_line(1U, text);
    sprintf(text, "RateT:%6.3f", (double)control_yaw_rate_target);
    diag_show_line(2U, text);
    sprintf(text, "RateC:%6.3f", (double)control_yaw_rate_current);
    diag_show_line(3U, text);
    sprintf(text, "RateO:%7.1f", (double)control_yaw_rate_output);
    diag_show_line(4U, text);
    sprintf(text, "M1 P:%6.1f", (double)wheel_left_front_pid.p_term);
    diag_show_line(5U, text);
    sprintf(text, "M1 I:%6.1f O:%6.1f", (double)wheel_left_front_pid.i_term,
            (double)wheel_left_front_pid.output);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* 诊断页：AirComm通信状态（在线/ACK/同步统计） */
static void diag_air_function(void)
{
    char text[32];
    air_comm_stats_t stats;
    menu_air_sync_status_t sync_status;

    air_comm_car_get_stats(&stats);
    menu_get_air_sync_status(&sync_status);

    diag_begin();
    sprintf(text, "Air online:%u", (unsigned int)stats.online_status);
    diag_show_line(0U, text);
    sprintf(text, "%s R:%u", diag_air_sync_mode_text(sync_status.mode),
            (unsigned int)sync_status.reason);
    diag_show_line(1U, text);
    sprintf(text, "Idx:%u Pend:%u", (unsigned int)sync_status.active_index,
            (unsigned int)stats.pending_ack);
    diag_show_line(2U, text);
    sprintf(text, "Last T:%u R:%u", (unsigned int)stats.last_ack_type,
            (unsigned int)stats.last_ack_result);
    diag_show_line(3U, text);
    sprintf(text, "St:%u Send:%lu", (unsigned int)stats.last_ack_status,
            (unsigned long)sync_status.send_count);
    diag_show_line(4U, text);
    sprintf(text, "OK:%lu Fail:%lu", (unsigned long)sync_status.ok_count,
            (unsigned long)sync_status.fail_count);
    diag_show_line(5U, text);
    sprintf(text, "TO:%lu FIdx:%u", (unsigned long)sync_status.timeout_count,
            (unsigned int)sync_status.last_failed_index);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}
