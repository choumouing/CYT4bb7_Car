/*********************************************************************************************************************
* 菜单用户配置实现文件 - 参数集中管理
*
* 功能：注册四电机速度环参数，并保留Flash存档读取/保存入口
********************************************************************************************************************/

#include "menu_config.h"

//====================================================参数变量区====================================================
// 轮速PID参数（四个电机共用）
float wheel_kp = 3.6f;                  // 比例系数
float wheel_ki = 0.27f;                  // 积分系数
float wheel_kd = 0.0f;                  // 微分系数
float wheel_output_limit = 5000.0f;     // 输出限幅 (PWM)

//====================================================用户函数声明====================================================
float yaw_angle_kp = 2.10f;
float yaw_angle_ki = 0.0f;
float yaw_angle_kd = 0.80f;
float yaw_angle_i_limit = 1.0f;
float yaw_angle_output_limit = 1.95f;

float yaw_rate_kp = 90.0f;
float yaw_rate_ki = 1.6f;
float yaw_rate_kd = 0.0f;
float yaw_rate_i_limit = 60.0f;
float yaw_rate_output_limit = 1000.0f;

float uwb_follow_deadband_x_cm = 4.0f;
float uwb_follow_deadband_y_cm = 5.0f;
float uwb_follow_output_limit = 500.0f;
float uwb_follow_i_limit = 0.0f;
float uwb_follow_x_kp = 2.2f;
float uwb_follow_x_ki = 0.0f;
float uwb_follow_x_kd = 1.0f;
float uwb_follow_y_kp = 1.9f;
float uwb_follow_y_ki = 0.0f;
float uwb_follow_y_kd = 0.8f;

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
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t yaw_rate_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 4},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 5},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 6},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 7},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 8},
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
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 9},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 10},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 11},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 12},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 13},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t uwb_follow_pid_menu[] = {
    {"DeadX", MENU_TYPE_PARAMETER, .param_index = 14},
    {"DeadY", MENU_TYPE_PARAMETER, .param_index = 15},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 16},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 17},
    {"XKp", MENU_TYPE_PARAMETER, .param_index = 18},
    {"XKi", MENU_TYPE_PARAMETER, .param_index = 19},
    {"XKd", MENU_TYPE_PARAMETER, .param_index = 20},
    {"YKp", MENU_TYPE_PARAMETER, .param_index = 21},
    {"YKi", MENU_TYPE_PARAMETER, .param_index = 22},
    {"YKd", MENU_TYPE_PARAMETER, .param_index = 23},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_param_menu[] = {
    {"MinArea", MENU_TYPE_AIR_PARAMETER, .param_index = 0},
    {"HoldMs", MENU_TYPE_AIR_PARAMETER, .param_index = 1},
    {"XBias", MENU_TYPE_AIR_PARAMETER, .param_index = 2},
    {"YBias", MENU_TYPE_AIR_PARAMETER, .param_index = 3},
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
    {"Air Param", MENU_TYPE_SUBMENU, .submenu = air_param_menu},
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
    {"UWB PID", MENU_TYPE_SUBMENU, .submenu = uwb_follow_pid_menu},
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

    menu_register_param(&uwb_follow_deadband_x_cm, 1.0f, 0.0f, 50.0f);
    menu_register_param(&uwb_follow_deadband_y_cm, 1.0f, 0.0f, 50.0f);
    menu_register_param(&uwb_follow_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&uwb_follow_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&uwb_follow_x_kp, 0.1f, 0.0f, 50.0f);
    menu_register_param(&uwb_follow_x_ki, 0.01f, 0.0f, 50.0f);
    menu_register_param(&uwb_follow_x_kd, 0.1f, 0.0f, 50.0f);
    menu_register_param(&uwb_follow_y_kp, 0.1f, 0.0f, 50.0f);
    menu_register_param(&uwb_follow_y_ki, 0.01f, 0.0f, 50.0f);
    menu_register_param(&uwb_follow_y_kd, 0.1f, 0.0f, 50.0f);

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
        menu_show_success("Air Queued");
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
    sprintf(text, "Odo F:%7.3f", (double)g_odometer.forward_distance);
    diag_show_line(0U, text);
    sprintf(text, "Odo S:%7.3f", (double)g_odometer.strafe_distance);
    diag_show_line(1U, text);
    sprintf(text, "Travel:%7.3f", (double)g_odometer.travel_distance);
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
    sprintf(text, "Pend:%u Type:%u", (unsigned int)stats.pending_ack,
            (unsigned int)stats.pending_ack_type);
    diag_show_line(1U, text);
    sprintf(text, "Last T:%u R:%u", (unsigned int)stats.last_ack_type,
            (unsigned int)stats.last_ack_result);
    diag_show_line(2U, text);
    sprintf(text, "Status:%u Dirty:%u", (unsigned int)stats.last_ack_status,
            (unsigned int)sync_status.dirty_count);
    diag_show_line(3U, text);
    sprintf(text, "Send:%lu", (unsigned long)sync_status.send_count);
    diag_show_line(4U, text);
    sprintf(text, "OK:%lu Fail:%lu", (unsigned long)sync_status.ok_count,
            (unsigned long)sync_status.fail_count);
    diag_show_line(5U, text);
    sprintf(text, "FailIdx:%u", (unsigned int)sync_status.last_failed_index);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}
