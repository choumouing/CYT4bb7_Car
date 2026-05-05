/*********************************************************************************************************************
* 菜单用户配置实现文件 - 参数集中管理
* 功能：定义菜单树和参数注册，集中管理所有菜单可调参数
* 版本：v3.1 (多航点导航系统 + 校准表参数)
* 日期：2025-12-15
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "menu_config.h"
#include "zf_device_key.h"
#include "../navigation/navigation_task.h"
#include "../navigation/waypoint_manager.h"
#include "../Attitude/IMU_TOP.h"
#include "../encoder/encoder_control.h"
#include "../motor/motor.h"
#include <stdio.h>

//====================================================参数变量区====================================================
// S曲线速度规划参数
float s_curve_v_max = 1000.0f;          // 最大速度 (mm/s)
float s_curve_a_max = 1200.0f;          // 最大加速度 (mm/s²)
float s_curve_j_max = 4000.0f;          // 最大Jerk (mm/s³)

// S曲线算法参数
float s_curve_max_iter = 30.0f;         // vlim迭代最大次数
float s_curve_conv_tol = 1.0f;          // 位移收敛容差 (mm)
float s_curve_min_dist = 50.0f;         // 极短距离阈值 (mm)

// 导航任务参数
float nav_dist_tol = 5.0f;             // 到达目标距离容差 (mm)
// 航向角PID参数
float yaw_kp_2 = 0.0f;                  // 二次项比例系数 [(rad/s)/°²]
float yaw_kp_1 = 0.3f;                  // 一次项比例系数 [(rad/s)/°]
float yaw_ki = 0.00f;                   // 积分系数 [(rad/s)/°]
float yaw_kd = 0.0f;                    // 微分系数 [(rad/s)/°]
float yaw_i_limit = 0.0f;               // 积分限幅 (°)
float yaw_output_limit = 1000.0f;       // 输出限幅 (rad/s)
float yaw_pos_d_lpf = 0.0f;             // 微分低通滤波系数 [0-1]
float yaw_tolerance = 1.0f;             // 航向角容差 (°)

// Yaw角速度PID参数
float yaw_rate_kp = 5.0f;               // 比例系数 [(pulse/5ms)/(rad/s)]
float yaw_rate_ki = 0.01f;              // 积分系数 [(pulse/5ms)/(rad/s)]
float yaw_rate_kd = 0.0f;               // 微分系数 [(pulse/5ms)/(rad/s)]
float yaw_rate_i_limit = 20.0f;         // 积分限幅 (rad/s)
float yaw_rate_output_limit = 1000.0f;  // 输出限幅 (pulse/5ms)
float yaw_rate_d_lpf = 0.0f;            // 微分低通滤波系数 [0-1]

// 轮速PID参数（增量式，共用）
float wheel_kp = 4.5f;                  // 比例系数
float wheel_ki = 0.8f;                  // 积分系数
float wheel_kd = 0.0f;                  // 微分系数
float wheel_output_limit = 5000.0f;     // 输出限幅 (PWM)
float wheel_inc_i_lpf = 0.1f;           // 积分低通滤波系数

// UWB位置环X方向PID参数（标签右向相对误差 -> 横移速度）
float uwb_x_kp = 1.0f;
float uwb_x_ki = 0.00f;
float uwb_x_kd = 0.01f;
float uwb_x_i_limit = 200.0f;
float uwb_x_output_limit = 1500.0f;
float uwb_x_d_lpf = 0.90f;

// UWB位置环Y方向PID参数（标签前向相对误差 -> 前进速度）
float uwb_y_kp = 0.85f;
float uwb_y_ki = 0.00f;
float uwb_y_kd = 0.01f;
float uwb_y_i_limit = 200.0f;
float uwb_y_output_limit = 1500.0f;
float uwb_y_d_lpf = 0.90f;

// 发车标志
float is_car_running = 0.0f;            // 发车状态 (0:停止 1:运行)

// 多航点导航管理器
waypoint_manager_t g_waypoint_mgr = {0};  // 航点管理器全局实例

// X方向校准表参数（5个标定点的PULSE_PER_MM）
float calib_x_1000 = 11.7f;
float calib_x_2000 = 12.2f;
float calib_x_3000 = 12.2f;
float calib_x_4000 = 12.0f;
float calib_x_5000 = 12.0f;

// Y方向校准表参数（5个标定点的PULSE_PER_MM）
float calib_y_1000 = 13.8f;
float calib_y_2000 = 13.8f;
float calib_y_3000 = 13.7f;
float calib_y_4000 = 13.7f;
float calib_y_5000 = 13.6f;

// 航点参数暴露变量（用于菜单配置）
float wp0_x = 0.0f, wp0_y = 0.0f, wp0_yaw = 0.0f, wp0_en = 0.0f;
float wp1_x = 0.0f, wp1_y = 0.0f, wp1_yaw = 0.0f, wp1_en = 0.0f;
float wp2_x = 0.0f, wp2_y = 0.0f, wp2_yaw = 0.0f, wp2_en = 0.0f;
float wp3_x = 0.0f, wp3_y = 0.0f, wp3_yaw = 0.0f, wp3_en = 0.0f;
float wp4_x = 0.0f, wp4_y = 0.0f, wp4_yaw = 0.0f, wp4_en = 0.0f;
float wp5_x = 0.0f, wp5_y = 0.0f, wp5_yaw = 0.0f, wp5_en = 0.0f;
float wp6_x = 0.0f, wp6_y = 0.0f, wp6_yaw = 0.0f, wp6_en = 0.0f;
float wp7_x = 0.0f, wp7_y = 0.0f, wp7_yaw = 0.0f, wp7_en = 0.0f;
float wp8_x = 0.0f, wp8_y = 0.0f, wp8_yaw = 0.0f, wp8_en = 0.0f;
float wp9_x = 0.0f, wp9_y = 0.0f, wp9_yaw = 0.0f, wp9_en = 0.0f;

//====================================================用户函数声明====================================================
void load_slot_0_function(void);
void load_slot_1_function(void);
void save_slot_0_function(void);
void save_slot_1_function(void);
void start_mission_function(void);
void stop_mission_function(void);
void show_info_function(void);

//====================================================菜单树定义====================================================

// S曲线参数子菜单
static menu_item_t s_curve_menu[] = {
    {"V_max", MENU_TYPE_PARAMETER, .param_index = 0},
    {"A_max", MENU_TYPE_PARAMETER, .param_index = 1},
    {"J_max", MENU_TYPE_PARAMETER, .param_index = 2},
    {"MaxIter", MENU_TYPE_PARAMETER, .param_index = 3},
    {"ConvTol", MENU_TYPE_PARAMETER, .param_index = 4},
    {"MinDist", MENU_TYPE_PARAMETER, .param_index = 5},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// X方向校准表子菜单（前置声明）
static menu_item_t calib_x_menu[];

// Y方向校准表子菜单（前置声明）
static menu_item_t calib_y_menu[];

// 导航参数子菜单
static menu_item_t nav_menu[] = {
    {"DistTol", MENU_TYPE_PARAMETER, .param_index = 6},
    {"Calib X", MENU_TYPE_SUBMENU, .submenu = calib_x_menu},
    {"Calib Y", MENU_TYPE_SUBMENU, .submenu = calib_y_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 航向角PID子菜单
static menu_item_t yaw_pid_menu[] = {
    {"Kp2", MENU_TYPE_PARAMETER, .param_index = 7},
    {"Kp1", MENU_TYPE_PARAMETER, .param_index = 8},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 9},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 10},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 11},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 12},
    {"DLpf", MENU_TYPE_PARAMETER, .param_index = 13},
    {"YawTol", MENU_TYPE_PARAMETER, .param_index = 14},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// Yaw角速度PID子菜单
static menu_item_t yaw_rate_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 70},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 71},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 72},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 73},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 74},
    {"DLpf", MENU_TYPE_PARAMETER, .param_index = 75},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 轮速PID子菜单（增量式）
static menu_item_t wheel_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 15},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 16},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 17},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 18},
    {"ILpf", MENU_TYPE_PARAMETER, .param_index = 19},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// UWB位置环X方向PID子菜单
static menu_item_t uwb_x_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 76},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 77},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 78},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 79},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 80},
    {"DLpf", MENU_TYPE_PARAMETER, .param_index = 81},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// UWB位置环Y方向PID子菜单
static menu_item_t uwb_y_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 82},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 83},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 84},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 85},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 86},
    {"DLpf", MENU_TYPE_PARAMETER, .param_index = 87},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// UWB位置环PID总菜单
static menu_item_t uwb_pid_menu[] = {
    {"X Pos PID", MENU_TYPE_SUBMENU, .submenu = uwb_x_pid_menu},
    {"Y Pos PID", MENU_TYPE_SUBMENU, .submenu = uwb_y_pid_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 航点配置子菜单
static menu_item_t wp0_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 20},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 21},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 22},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 23},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t wp1_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 24},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 25},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 26},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 27},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t wp2_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 28},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 29},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 30},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 31},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t wp3_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 32},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 33},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 34},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 35},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t wp4_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 36},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 37},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 38},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 39},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t wp5_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 40},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 41},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 42},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 43},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t wp6_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 44},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 45},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 46},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 47},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t wp7_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 48},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 49},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 50},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 51},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t wp8_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 52},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 53},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 54},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 55},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t wp9_menu[] = {
    {"X", MENU_TYPE_PARAMETER, .param_index = 56},
    {"Y", MENU_TYPE_PARAMETER, .param_index = 57},
    {"Yaw", MENU_TYPE_PARAMETER, .param_index = 58},
    {"Enable", MENU_TYPE_PARAMETER, .param_index = 59},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 所有航点子菜单
static menu_item_t waypoints_menu[] = {
    {"Waypoint0", MENU_TYPE_SUBMENU, .submenu = wp0_menu},
    {"Waypoint1", MENU_TYPE_SUBMENU, .submenu = wp1_menu},
    {"Waypoint2", MENU_TYPE_SUBMENU, .submenu = wp2_menu},
    {"Waypoint3", MENU_TYPE_SUBMENU, .submenu = wp3_menu},
    {"Waypoint4", MENU_TYPE_SUBMENU, .submenu = wp4_menu},
    {"Waypoint5", MENU_TYPE_SUBMENU, .submenu = wp5_menu},
    {"Waypoint6", MENU_TYPE_SUBMENU, .submenu = wp6_menu},
    {"Waypoint7", MENU_TYPE_SUBMENU, .submenu = wp7_menu},
    {"Waypoint8", MENU_TYPE_SUBMENU, .submenu = wp8_menu},
    {"Waypoint9", MENU_TYPE_SUBMENU, .submenu = wp9_menu},
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

// X方向校准表子菜单
static menu_item_t calib_x_menu[] = {
    {"X_1000", MENU_TYPE_PARAMETER, .param_index = 60},
    {"X_2000", MENU_TYPE_PARAMETER, .param_index = 61},
    {"X_3000", MENU_TYPE_PARAMETER, .param_index = 62},
    {"X_4000", MENU_TYPE_PARAMETER, .param_index = 63},
    {"X_5000", MENU_TYPE_PARAMETER, .param_index = 64},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// Y方向校准表子菜单
static menu_item_t calib_y_menu[] = {
    {"Y_1000", MENU_TYPE_PARAMETER, .param_index = 65},
    {"Y_2000", MENU_TYPE_PARAMETER, .param_index = 66},
    {"Y_3000", MENU_TYPE_PARAMETER, .param_index = 67},
    {"Y_4000", MENU_TYPE_PARAMETER, .param_index = 68},
    {"Y_5000", MENU_TYPE_PARAMETER, .param_index = 69},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 主菜单
static menu_item_t main_menu[] = {
    {"Waypoints", MENU_TYPE_SUBMENU, .submenu = waypoints_menu},
    {"Start", MENU_TYPE_FUNCTION, .function = start_mission_function},
    {"Stop", MENU_TYPE_FUNCTION, .function = stop_mission_function},
    {"S Curve", MENU_TYPE_SUBMENU, .submenu = s_curve_menu},
    {"Navigation", MENU_TYPE_SUBMENU, .submenu = nav_menu},
    {"Yaw PID", MENU_TYPE_SUBMENU, .submenu = yaw_pid_menu},
    {"Yaw Rate PID", MENU_TYPE_SUBMENU, .submenu = yaw_rate_pid_menu},
    {"Wheel PID", MENU_TYPE_SUBMENU, .submenu = wheel_pid_menu},
    {"UWB PID", MENU_TYPE_SUBMENU, .submenu = uwb_pid_menu},
    {"Show Info", MENU_TYPE_FUNCTION, .function = show_info_function},
    {"Load Slot", MENU_TYPE_SUBMENU, .submenu = load_slot_menu},
    {"Save Slot", MENU_TYPE_SUBMENU, .submenu = save_slot_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

//====================================================用户配置初始化====================================================
/**
 * @brief 初始化菜单配置
 */
void menu_config_init(void)
{
    // 注册S曲线参数
    menu_register_param(&s_curve_v_max, 100.0f, 100.0f, 10000.0f);     // 参数0
    menu_register_param(&s_curve_a_max, 100.0f, 100.0f, 10000.0f);    // 参数1
    menu_register_param(&s_curve_j_max, 100.0f, 100.0f, 100000.0f);  // 参数2
    menu_register_param(&s_curve_max_iter, 1.0f, 5.0f, 50.0f);       // 参数3
    menu_register_param(&s_curve_conv_tol, 0.1f, 0.1f, 1000.0f);     // 参数4
    menu_register_param(&s_curve_min_dist, 0.1f, 1.0f, 1000.0f);     // 参数5

    // 注册导航参数
    menu_register_param(&nav_dist_tol, 5.0f, 10.0f, 10000.0f);       // 参数6

    // 注册航向角PID参数
    menu_register_param(&yaw_kp_2, 0.01f, 0.0f, 100.0f);             // 参数7
    menu_register_param(&yaw_kp_1, 0.01f, 0.0f, 100.0f);             // 参数8
    menu_register_param(&yaw_ki, 0.01f, 0.0f, 100.0f);               // 参数9
    menu_register_param(&yaw_kd, 0.01f, 0.0f, 100.0f);               // 参数10
    menu_register_param(&yaw_i_limit, 0.1f, 0.0f, 10.0f);            // 参数11
    menu_register_param(&yaw_output_limit, 0.1f, 0.5f, 5000.0f);     // 参数12
    menu_register_param(&yaw_pos_d_lpf, 0.01f, 0.0f, 1.0f);          // 参数13
    menu_register_param(&yaw_tolerance, 1.0f, 0.1f, 180.0f);         // 参数14

    // 注册轮速PID参数（增量式）
    menu_register_param(&wheel_kp, 0.1f, 0.0f, 100.0f);             // 参数15
    menu_register_param(&wheel_ki, 0.1f, 0.0f, 100.0f);              // 参数16
    menu_register_param(&wheel_kd, 0.1f, 0.0f, 100.0f);              // 参数17
    menu_register_param(&wheel_output_limit, 100.0f, 1000.0f, 10000.0f); // 参数18
    menu_register_param(&wheel_inc_i_lpf, 0.01f, 0.0f, 1.0f);        // 参数19

    // 注册航点0参数
    menu_register_param(&wp0_x, 100.0f, -50000.0f, 50000.0f);        // 参数20
    menu_register_param(&wp0_y, 100.0f, -50000.0f, 50000.0f);        // 参数21
    menu_register_param(&wp0_yaw, 10.0f, -180.0f, 180.0f);           // 参数22
    menu_register_param(&wp0_en, 1.0f, 0.0f, 1.0f);                  // 参数23

    // 注册航点1参数
    menu_register_param(&wp1_x, 100.0f, -50000.0f, 50000.0f);        // 参数24
    menu_register_param(&wp1_y, 100.0f, -50000.0f, 50000.0f);        // 参数25
    menu_register_param(&wp1_yaw, 10.0f, -180.0f, 180.0f);           // 参数26
    menu_register_param(&wp1_en, 1.0f, 0.0f, 1.0f);                  // 参数27

    // 注册航点2参数
    menu_register_param(&wp2_x, 100.0f, -50000.0f, 50000.0f);        // 参数28
    menu_register_param(&wp2_y, 100.0f, -50000.0f, 50000.0f);        // 参数29
    menu_register_param(&wp2_yaw, 10.0f, -180.0f, 180.0f);           // 参数30
    menu_register_param(&wp2_en, 1.0f, 0.0f, 1.0f);                  // 参数31

    // 注册航点3参数
    menu_register_param(&wp3_x, 100.0f, -50000.0f, 50000.0f);        // 参数32
    menu_register_param(&wp3_y, 100.0f, -50000.0f, 50000.0f);        // 参数33
    menu_register_param(&wp3_yaw, 10.0f, -180.0f, 180.0f);           // 参数34
    menu_register_param(&wp3_en, 1.0f, 0.0f, 1.0f);                  // 参数35

    // 注册航点4参数
    menu_register_param(&wp4_x, 100.0f, -50000.0f, 50000.0f);        // 参数36
    menu_register_param(&wp4_y, 100.0f, -50000.0f, 50000.0f);        // 参数37
    menu_register_param(&wp4_yaw, 10.0f, -180.0f, 180.0f);           // 参数38
    menu_register_param(&wp4_en, 1.0f, 0.0f, 1.0f);                  // 参数39

    // 注册航点5参数
    menu_register_param(&wp5_x, 100.0f, -50000.0f, 50000.0f);        // 参数40
    menu_register_param(&wp5_y, 100.0f, -50000.0f, 50000.0f);        // 参数41
    menu_register_param(&wp5_yaw, 10.0f, -180.0f, 180.0f);           // 参数42
    menu_register_param(&wp5_en, 1.0f, 0.0f, 1.0f);                  // 参数43

    // 注册航点6参数
    menu_register_param(&wp6_x, 100.0f, -50000.0f, 50000.0f);        // 参数44
    menu_register_param(&wp6_y, 100.0f, -50000.0f, 50000.0f);        // 参数45
    menu_register_param(&wp6_yaw, 10.0f, -180.0f, 180.0f);           // 参数46
    menu_register_param(&wp6_en, 1.0f, 0.0f, 1.0f);                  // 参数47

    // 注册航点7参数
    menu_register_param(&wp7_x, 100.0f, -50000.0f, 50000.0f);        // 参数48
    menu_register_param(&wp7_y, 100.0f, -50000.0f, 50000.0f);        // 参数49
    menu_register_param(&wp7_yaw, 10.0f, -180.0f, 180.0f);           // 参数50
    menu_register_param(&wp7_en, 1.0f, 0.0f, 1.0f);                  // 参数51

    // 注册航点8参数
    menu_register_param(&wp8_x, 100.0f, -50000.0f, 50000.0f);        // 参数52
    menu_register_param(&wp8_y, 100.0f, -50000.0f, 50000.0f);        // 参数53
    menu_register_param(&wp8_yaw, 10.0f, -180.0f, 180.0f);           // 参数54
    menu_register_param(&wp8_en, 1.0f, 0.0f, 1.0f);                  // 参数55

    // 注册航点9参数
    menu_register_param(&wp9_x, 100.0f, -50000.0f, 50000.0f);        // 参数56
    menu_register_param(&wp9_y, 100.0f, -50000.0f, 50000.0f);        // 参数57
    menu_register_param(&wp9_yaw, 10.0f, -180.0f, 180.0f);           // 参数58
    menu_register_param(&wp9_en, 1.0f, 0.0f, 1.0f);                  // 参数59

    // 注册X方向校准表参数
    menu_register_param(&calib_x_1000, 0.1f, 8.0f, 20.0f);          // 参数60
    menu_register_param(&calib_x_2000, 0.1f, 8.0f, 20.0f);          // 参数61
    menu_register_param(&calib_x_3000, 0.1f, 8.0f, 20.0f);          // 参数62
    menu_register_param(&calib_x_4000, 0.1f, 8.0f, 20.0f);          // 参数63
    menu_register_param(&calib_x_5000, 0.1f, 8.0f, 20.0f);          // 参数64

    // 注册Y方向校准表参数
    menu_register_param(&calib_y_1000, 0.001f, 8.0f, 20.0f);         // 参数65
    menu_register_param(&calib_y_2000, 0.001f, 8.0f, 20.0f);         // 参数66
    menu_register_param(&calib_y_3000, 0.001f, 8.0f, 20.0f);         // 参数67
    menu_register_param(&calib_y_4000, 0.001f, 8.0f, 20.0f);         // 参数68
    menu_register_param(&calib_y_5000, 0.001f, 8.0f, 20.0f);         // 参数69

    // 注册Yaw角速度PID参数
    menu_register_param(&yaw_rate_kp, 0.1f, 0.0f, 500.0f);            // 参数70
    menu_register_param(&yaw_rate_ki, 0.1f, 0.0f, 500.0f);            // 参数71
    menu_register_param(&yaw_rate_kd, 0.1f, 0.0f, 500.0f);            // 参数72
    menu_register_param(&yaw_rate_i_limit, 0.1f, 0.0f, 1000.0f);      // 参数73
    menu_register_param(&yaw_rate_output_limit, 1.0f, 0.0f, 5000.0f); // 参数74
    menu_register_param(&yaw_rate_d_lpf, 0.01f, 0.0f, 1.0f);          // 参数75

    // 注册UWB位置环X方向PID参数
    menu_register_param(&uwb_x_kp, 0.01f, 0.0f, 10.0f);               // 参数76
    menu_register_param(&uwb_x_ki, 0.01f, 0.0f, 10.0f);               // 参数77
    menu_register_param(&uwb_x_kd, 0.01f, 0.0f, 10.0f);               // 参数78
    menu_register_param(&uwb_x_i_limit, 10.0f, 0.0f, 5000.0f);        // 参数79
    menu_register_param(&uwb_x_output_limit, 10.0f, 0.0f, 2000.0f);   // 参数80
    menu_register_param(&uwb_x_d_lpf, 0.01f, 0.0f, 1.0f);             // 参数81

    // 注册UWB位置环Y方向PID参数
    menu_register_param(&uwb_y_kp, 0.01f, 0.0f, 10.0f);               // 参数82
    menu_register_param(&uwb_y_ki, 0.01f, 0.0f, 10.0f);               // 参数83
    menu_register_param(&uwb_y_kd, 0.01f, 0.0f, 10.0f);               // 参数84
    menu_register_param(&uwb_y_i_limit, 10.0f, 0.0f, 5000.0f);        // 参数85
    menu_register_param(&uwb_y_output_limit, 10.0f, 0.0f, 2000.0f);   // 参数86
    menu_register_param(&uwb_y_d_lpf, 0.01f, 0.0f, 1.0f);             // 参数87

    // 设置根菜单
    menu_set_root(main_menu);
}

//====================================================用户函数实现====================================================

/**
 * @brief 加载存档0
 */
void load_slot_0_function(void)
{
    menu_load_slot(0);
}

/**
 * @brief 加载存档1
 */
void load_slot_1_function(void)
{
    menu_load_slot(1);
}

/**
 * @brief 保存到存档0
 */
void save_slot_0_function(void)
{
    menu_save_slot(0);
}

/**
 * @brief 保存到存档1
 */
void save_slot_1_function(void)
{
    menu_save_slot(1);
}

/**
 * @brief 启动多航点导航任务
 */
void start_mission_function(void)
{
    extern odometry_t odometry;
    extern navigation_task_t navigation_task;

    waypoint_mission_stop(&g_waypoint_mgr);
    navigation_stop(&navigation_task);
    odometry_reset_pose(&odometry, -g_euler.yaw);

    if (waypoint_mission_start(&g_waypoint_mgr, &navigation_task) == 0) {
        menu_show_success("Waypoint Start");
    } else {
        menu_show_error("No Waypoint");
    }
}

/**
 * @brief 停止任务
 */
void stop_mission_function(void)
{
    extern navigation_task_t navigation_task;

    waypoint_mission_stop(&g_waypoint_mgr);
    navigation_stop(&navigation_task);
    menu_show_success("Waypoint Stop");
}

/**
 * @brief 显示信息 - 实时显示多航点导航状态
 */
void show_info_function(void)
{
    extern odometry_t odometry;
    extern navigation_task_t navigation_task;

    char line_buf[32];

    // 初始化显示配置
    ips114_set_color(UI_COLOR_NORMAL, UI_COLOR_BG);
    ips114_set_font(UI_FONT_NORMAL);
    ips114_clear();

    // 显示标题
    ips114_show_string(0, 0, "=== Waypoint Nav ===");

    // 显示里程计信息
    sprintf(line_buf, "Pos X: %.1f mm", odometry.pos_x);
    ips114_show_string(0, 20, line_buf);

    sprintf(line_buf, "Pos Y: %.1f mm", odometry.pos_y);
    ips114_show_string(0, 35, line_buf);

    sprintf(line_buf, "Yaw: %.1f deg", odometry.pos_theta);
    ips114_show_string(0, 50, line_buf);

    // 显示多航点导航状态
    sprintf(line_buf, "State:%d Active:%d", navigation_get_state(&navigation_task),
            waypoint_is_mission_active(&g_waypoint_mgr));
    ips114_show_string(0, 70, line_buf);

    sprintf(line_buf, "WP:%d/%d", waypoint_get_current_index(&g_waypoint_mgr),
            waypoint_get_enabled_count(&g_waypoint_mgr));
    ips114_show_string(0, 90, line_buf);

    sprintf(line_buf, "TarX: %.0f", navigation_task.target_x);
    ips114_show_string(0, 105, line_buf);

    sprintf(line_buf, "TarY: %.0f", navigation_task.target_y);
    ips114_show_string(0, 120, line_buf);

    // 等待按键退出
    while (key_get_state(KEY_1) == KEY_SHORT_PRESS);
    while (key_get_state(KEY_1) != KEY_SHORT_PRESS);
}
