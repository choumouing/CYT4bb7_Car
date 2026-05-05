/*********************************************************************************************************************
* 菜单用户配置实现文件 - 参数集中管理
*
* 功能：注册四电机速度环参数，并保留Flash存档读取/保存入口
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "menu_config.h"

//====================================================参数变量区====================================================
// 轮速PID参数（四个电机共用）
float wheel_kp = 3.5f;                  // 比例系数
float wheel_ki = 0.4f;                  // 积分系数
float wheel_kd = 0.0f;                  // 微分系数
float wheel_output_limit = 5000.0f;     // 输出限幅 (PWM)

//====================================================用户函数声明====================================================
static void load_slot_0_function(void);
static void load_slot_1_function(void);
static void save_slot_0_function(void);
static void save_slot_1_function(void);

//====================================================菜单树定义====================================================
// 轮速PID子菜单（增量式）
static menu_item_t wheel_pid_menu[] = {
    {"Kp", MENU_TYPE_PARAMETER, .param_index = 0},
    {"Ki", MENU_TYPE_PARAMETER, .param_index = 1},
    {"Kd", MENU_TYPE_PARAMETER, .param_index = 2},
    {"OutLimit", MENU_TYPE_PARAMETER, .param_index = 3},
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
static menu_item_t main_menu[] = {
    {"Wheel PID", MENU_TYPE_SUBMENU, .submenu = wheel_pid_menu},
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
