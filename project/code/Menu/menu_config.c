/*********************************************************************************************************************
* 菜单用户配置实现文件 - 参数集中管理
*
* 功能：注册四电机速度环参数，并保留Flash存档读取/保存入口
********************************************************************************************************************/

#include "menu_config.h"

#define MENU_CAR_S_CURVE_MAX_ITER_INDEX  (55U)
#define MENU_CAR_S_CURVE_CONV_TOL_INDEX  (56U)
#define MENU_CAR_S_CURVE_MIN_DIST_INDEX  (57U)
#define MENU_CAR_EXPECTED_PARAM_COUNT     (58U)

#if (MENU_CAR_EXPECTED_PARAM_COUNT > MENU_MAX_PARAMS)
#error "Car menu parameter count exceeds MENU_MAX_PARAMS"
#endif

//====================================================参数变量区====================================================
// 轮速PID参数（四个电机共用）
float wheel_kp = 2.3f;                  // 比例系数
float wheel_ki = 0.08f;                  // 积分系数
float wheel_kd = 0.0f;                  // 微分系数
float wheel_output_limit = 5000.0f;     // 输出限幅 (PWM)
float wheel_i_limit = 2500.0f;          // 积分限幅

//====================================================用户函数声明====================================================
float yaw_angle_kp = 3.8f;
float yaw_angle_ki = 0.0f;
float yaw_angle_kd = 1.0f;
float yaw_angle_i_limit = 4.0f;
float yaw_angle_output_limit = 100.0f;

float yaw_rate_kp = 125.0f;
float yaw_rate_ki = 0.03f;
float yaw_rate_kd = 0.0f;
float yaw_rate_i_limit = 50.0f;
float yaw_rate_output_limit = 3000.0f;

float mode7_velocity_smooth_tau_s = 0.12f;
float mode7_velocity_output_limit = 650.0f;
float mode7_velocity_pid_output_limit = 250.0f;
float mode7_velocity_i_limit = 0.0f;
float mode7_velocity_strafe_kp = 80.0f;
float mode7_velocity_strafe_ki = 0.0f;
float mode7_velocity_strafe_kd = 20.0f;
float mode7_velocity_forward_kp = 80.0f;
float mode7_velocity_forward_ki = 0.0f;
float mode7_velocity_forward_kd = 20.0f;

float mode5_velocity_smooth_tau_s = 0.12f;
float mode5_velocity_output_limit = 650.0f;
float mode5_velocity_pid_output_limit = 250.0f;
float mode5_velocity_i_limit = 0.0f;
float mode5_velocity_strafe_kp = 80.0f;
float mode5_velocity_strafe_ki = 0.0f;
float mode5_velocity_strafe_kd = 20.0f;
float mode5_velocity_forward_kp = 80.0f;
float mode5_velocity_forward_ki = 0.0f;
float mode5_velocity_forward_kd = 20.0f;

float mode2_velocity_smooth_tau_s = 0.12f;
float mode2_velocity_output_limit = 650.0f;
float mode2_velocity_pid_output_limit = 250.0f;
float mode2_velocity_i_limit = 0.0f;
float mode2_velocity_strafe_kp = 80.0f;
float mode2_velocity_strafe_ki = 0.0f;
float mode2_velocity_strafe_kd = 20.0f;
float mode2_velocity_forward_kp = 80.0f;
float mode2_velocity_forward_ki = 0.0f;
float mode2_velocity_forward_kd = 20.0f;

float mode8_velocity_smooth_tau_s = 0.12f;
float mode8_velocity_output_limit = 650.0f;
float mode8_velocity_pid_output_limit = 250.0f;
float mode8_velocity_i_limit = 0.0f;
float mode8_velocity_strafe_kp = 80.0f;
float mode8_velocity_strafe_ki = 0.0f;
float mode8_velocity_strafe_kd = 20.0f;
float mode8_velocity_forward_kp = 80.0f;
float mode8_velocity_forward_ki = 0.0f;
float mode8_velocity_forward_kd = 20.0f;

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
static void diag_air_state_function(void);
static void diag_air_attitude_function(void);
static void diag_air_rc_function(void);
static void diag_air_plan_function(void);
static void diag_air_comm_function(void);

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
    {"Load Car0", MENU_TYPE_FUNCTION, .function = load_slot_0_function},
    {"Load Car1", MENU_TYPE_FUNCTION, .function = load_slot_1_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

// 保存存档子菜单
static menu_item_t save_slot_menu[] = {
    {"Save Car0", MENU_TYPE_FUNCTION, .function = save_slot_0_function},
    {"Save Car1", MENU_TYPE_FUNCTION, .function = save_slot_1_function},
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

static menu_item_t mode7_velocity_pid_menu[] = {
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

static menu_item_t mode5_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 35},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 36},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 37},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 38},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 39},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 40},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 41},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 42},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 43},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 44},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode2_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 45},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 46},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 47},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 48},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 49},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 50},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 51},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 52},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 53},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 54},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t mode8_velocity_pid_menu[] = {
    {"Smooth", MENU_TYPE_PARAMETER, .param_index = 25},
    {"CmdLimit", MENU_TYPE_PARAMETER, .param_index = 26},
    {"PidLimit", MENU_TYPE_PARAMETER, .param_index = 27},
    {"ILimit", MENU_TYPE_PARAMETER, .param_index = 28},
    {"SKp", MENU_TYPE_PARAMETER, .param_index = 29},
    {"SKi", MENU_TYPE_PARAMETER, .param_index = 30},
    {"SKd", MENU_TYPE_PARAMETER, .param_index = 31},
    {"FKp", MENU_TYPE_PARAMETER, .param_index = 32},
    {"FKi", MENU_TYPE_PARAMETER, .param_index = 33},
    {"FKd", MENU_TYPE_PARAMETER, .param_index = 34},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t s_curve_menu[] = {
    {"MaxIter", MENU_TYPE_PARAMETER, .param_index = MENU_CAR_S_CURVE_MAX_ITER_INDEX},
    {"ConvTol", MENU_TYPE_PARAMETER, .param_index = MENU_CAR_S_CURVE_CONV_TOL_INDEX},
    {"MinDist", MENU_TYPE_PARAMETER, .param_index = MENU_CAR_S_CURVE_MIN_DIST_INDEX},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

#define AIR_PARAM_MENU_STORAGE_COUNT \
    (MENU_AIR_EXPECTED_PARAM_COUNT + MENU_AIR_GROUP_COUNT)

static menu_item_t s_air_param_menu_storage[AIR_PARAM_MENU_STORAGE_COUNT];
static menu_item_t *s_air_group_menus[MENU_AIR_GROUP_COUNT];
static menu_item_t air_command_menu[2U];

enum
{
    AIR_PARAM_MENU_BASIC = 0,
    AIR_PARAM_MENU_GYRO,
    AIR_PARAM_MENU_ANGLE,
    AIR_PARAM_MENU_VELOCITY,
    AIR_PARAM_MENU_MODE7,
    AIR_PARAM_MENU_ESTIMATION,
    AIR_PARAM_MENU_MODE5,
    AIR_PARAM_MENU_MODE8_IMAGE,
    AIR_PARAM_MENU_MODE8_VELOCITY
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

static menu_item_t car_param_menu[] = {
    {"Wheel PID", MENU_TYPE_SUBMENU, .submenu = wheel_pid_menu},
    {"YawRate PID", MENU_TYPE_SUBMENU, .submenu = yaw_rate_pid_menu},
    {"YawAng PID", MENU_TYPE_SUBMENU, .submenu = yaw_angle_pid_menu},
    {"Mode2 Vel", MENU_TYPE_SUBMENU, .submenu = mode2_velocity_pid_menu},
    {"Mode5 Vel", MENU_TYPE_SUBMENU, .submenu = mode5_velocity_pid_menu},
    {"Mode7 Vel", MENU_TYPE_SUBMENU, .submenu = mode7_velocity_pid_menu},
    {"Mode8 Vel", MENU_TYPE_SUBMENU, .submenu = mode8_velocity_pid_menu},
    {"S Curve", MENU_TYPE_SUBMENU, .submenu = s_curve_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_param_menu[] = {
    {"Basic", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Gyro PID", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Angle PID", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Vel PID", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode7", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Estimation", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode5", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode8 Img", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"Mode8 Vel", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_diag_menu[] = {
    {"IMU", MENU_TYPE_DIAG_VIEW, .function = diag_imu_function},
    {"Encoder", MENU_TYPE_DIAG_VIEW, .function = diag_encoder_function},
    {"Position", MENU_TYPE_DIAG_VIEW, .function = diag_position_function},
    {"PID", MENU_TYPE_DIAG_VIEW, .function = diag_pid_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_diag_menu[] = {
    {"A_State", MENU_TYPE_DIAG_VIEW, .function = diag_air_state_function},
    {"A_Attitude", MENU_TYPE_DIAG_VIEW, .function = diag_air_attitude_function},
    {"A_RC", MENU_TYPE_DIAG_VIEW, .function = diag_air_rc_function},
    {"A_Plan", MENU_TYPE_DIAG_VIEW, .function = diag_air_plan_function},
    {"A_Comm", MENU_TYPE_DIAG_VIEW, .function = diag_air_comm_function},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t car_menu[] = {
    {"C_params", MENU_TYPE_SUBMENU, .submenu = car_param_menu},
    {"C_Diag", MENU_TYPE_SUBMENU, .submenu = car_diag_menu},
    {"C_Load", MENU_TYPE_SUBMENU, .submenu = load_slot_menu},
    {"C_Save", MENU_TYPE_SUBMENU, .submenu = save_slot_menu},
    {"C_BeaconRec", MENU_TYPE_SUBMENU, .submenu = NULL},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t air_menu[] = {
    {"A_params", MENU_TYPE_SUBMENU, .submenu = air_param_menu},
    {"A_Command", MENU_TYPE_SUBMENU, .submenu = air_command_menu},
    {"A_Diag", MENU_TYPE_SUBMENU, .submenu = air_diag_menu},
    {"Sync Air", MENU_TYPE_FUNCTION, .function = sync_air_function},
    {"A_Load", MENU_TYPE_SUBMENU, .submenu = load_air_slot_menu},
    {"A_Save", MENU_TYPE_SUBMENU, .submenu = save_air_slot_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static menu_item_t main_menu[] = {
    {"Car", MENU_TYPE_SUBMENU, .submenu = car_menu},
    {"Air", MENU_TYPE_SUBMENU, .submenu = air_menu},
    {"", MENU_TYPE_SUBMENU, .submenu = NULL}
};

static uint8 menu_build_air_param_menus(void)
{
    uint16 cursor = 0U;
    uint8 group;
    uint8 index;
    uint8 group_count;
    const menu_air_param_config_t *config;
    menu_item_t *item;

    memset(s_air_param_menu_storage, 0, sizeof(s_air_param_menu_storage));
    memset(s_air_group_menus, 0, sizeof(s_air_group_menus));

    for(group = 0U; group < MENU_AIR_GROUP_COUNT; group++)
    {
        s_air_group_menus[group] = &s_air_param_menu_storage[cursor];
        group_count = 0U;

        for(index = 0U; index < menu_get_air_param_count(); index++)
        {
            config = menu_get_air_param_config(index);
            if((config == NULL) || (config->group != group))
            {
                continue;
            }

            if((group_count >= MENU_MAX_ITEMS) ||
               (cursor >= AIR_PARAM_MENU_STORAGE_COUNT))
            {
                return 1U;
            }

            item = &s_air_param_menu_storage[cursor++];
            strncpy(item->name, config->name, sizeof(item->name) - 1U);
            item->name[sizeof(item->name) - 1U] = '\0';
            item->type = MENU_TYPE_AIR_PARAMETER;
            item->param_index = index;
            group_count++;
        }

        if(cursor >= AIR_PARAM_MENU_STORAGE_COUNT)
        {
            return 1U;
        }
        item = &s_air_param_menu_storage[cursor++];
        item->name[0] = '\0';
        item->type = MENU_TYPE_SUBMENU;
        item->submenu = NULL;
    }

    if((cursor != AIR_PARAM_MENU_STORAGE_COUNT) ||
       (menu_air_command_get_count() != 1U))
    {
        return 1U;
    }

    memset(air_command_menu, 0, sizeof(air_command_menu));
    strncpy(air_command_menu[0].name,
            menu_air_command_get_name(0U),
            sizeof(air_command_menu[0].name) - 1U);
    air_command_menu[0].type = MENU_TYPE_AIR_COMMAND;
    air_command_menu[0].param_index = 0U;
    air_command_menu[1].type = MENU_TYPE_SUBMENU;
    air_command_menu[1].submenu = NULL;

    air_param_menu[AIR_PARAM_MENU_BASIC].submenu = s_air_group_menus[MENU_AIR_GROUP_BASIC];
    air_param_menu[AIR_PARAM_MENU_GYRO].submenu = s_air_group_menus[MENU_AIR_GROUP_GYRO];
    air_param_menu[AIR_PARAM_MENU_ANGLE].submenu = s_air_group_menus[MENU_AIR_GROUP_ANGLE];
    air_param_menu[AIR_PARAM_MENU_VELOCITY].submenu = s_air_group_menus[MENU_AIR_GROUP_VELOCITY];
    air_param_menu[AIR_PARAM_MENU_MODE7].submenu = s_air_group_menus[MENU_AIR_GROUP_MODE7];
    air_param_menu[AIR_PARAM_MENU_ESTIMATION].submenu = s_air_group_menus[MENU_AIR_GROUP_ESTIMATION];
    air_param_menu[AIR_PARAM_MENU_MODE5].submenu = s_air_group_menus[MENU_AIR_GROUP_MODE5];
    air_param_menu[AIR_PARAM_MENU_MODE8_IMAGE].submenu = s_air_group_menus[MENU_AIR_GROUP_MODE8_IMAGE];
    air_param_menu[AIR_PARAM_MENU_MODE8_VELOCITY].submenu = s_air_group_menus[MENU_AIR_GROUP_MODE8_VELOCITY];

    return 0U;
}

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
    menu_register_param(&yaw_angle_output_limit, 0.1f, 0.0f, 100.0f);

    menu_register_param(&mode7_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode7_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode7_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode7_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode7_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode7_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&mode8_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode8_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode8_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode8_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode8_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode8_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&mode5_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode5_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode5_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode5_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode5_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode5_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&mode2_velocity_smooth_tau_s, 0.01f, 0.0f, 1.0f);
    menu_register_param(&mode2_velocity_output_limit, 10.0f, 0.0f, 1500.0f);
    menu_register_param(&mode2_velocity_pid_output_limit, 10.0f, 0.0f, 1000.0f);
    menu_register_param(&mode2_velocity_i_limit, 1.0f, 0.0f, 1000.0f);
    menu_register_param(&mode2_velocity_strafe_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_strafe_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_strafe_kd, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_forward_kp, 1.0f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_forward_ki, 0.01f, 0.0f, 500.0f);
    menu_register_param(&mode2_velocity_forward_kd, 1.0f, 0.0f, 500.0f);

    menu_register_param(&s_curve_max_iter, 1.0f, 5.0f, 200.0f);
    menu_register_param(&s_curve_conv_tol, 0.001f, 0.001f, 1000.0f);
    menu_register_param(&s_curve_min_dist, 0.1f, 1.0f, 1000.0f);

    if(menu_get_param_count() != MENU_CAR_EXPECTED_PARAM_COUNT)
    {
        menu_show_error("Car Menu Error");
        return;
    }

    menu_air_support_init();
    if(menu_build_air_param_menus() != 0U)
    {
        menu_show_error("Air Menu Error");
        return;
    }
    car_menu[4].submenu = beacon_position_recorder_get_menu();
    if(car_menu[4].submenu == NULL)
    {
        menu_show_error("Beacon Menu Error");
        return;
    }
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
        menu_show_progress("Air Loading");
    }
}

static void load_air_slot_1_function(void)
{
    if(menu_load_air_slot(1U) == 0U)
    {
        menu_show_progress("Air Loading");
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

        case MENU_AIR_SYNC_MODE_PULL:
            return "reading";

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

/* Air诊断页：飞行状态、TOF高度、位置估计速度和同步时间 */
static void diag_air_state_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air State");
    sprintf(text, "On:%u Fresh:%u",
            (unsigned int)air_comm_car_is_online(),
            (unsigned int)air_comm_car_is_run_data_fresh());
    diag_show_line(1U, text);
    sprintf(text, "State:%u", (unsigned int)(uint8)g_air_state);
    diag_show_line(2U, text);
    sprintf(text, "TOF:%7.1fmm", (double)g_air_tof_fused_height_mm);
    diag_show_line(3U, text);
    sprintf(text, "Vx:%8.3f", (double)g_air_pos_est_vel_x);
    diag_show_line(4U, text);
    sprintf(text, "Vy:%8.3f", (double)g_air_pos_est_vel_y);
    diag_show_line(5U, text);
    sprintf(text, "Sync:%7.0fms", (double)g_air_sync_time_ms);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* Air诊断页：姿态角和航向目标 */
static void diag_air_attitude_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air Attitude");
    sprintf(text, "Roll:%8.2f", (double)g_air_euler_roll);
    diag_show_line(1U, text);
    sprintf(text, "Pitch:%7.2f", (double)g_air_euler_pitch);
    diag_show_line(2U, text);
    sprintf(text, "Yaw:%9.2f", (double)g_air_euler_yaw);
    diag_show_line(3U, text);
    sprintf(text, "YawT:%8.2f", (double)g_air_yaw_angle_target_deg);
    diag_show_line(4U, text);
    sprintf(text, "State:%u", (unsigned int)(uint8)g_air_state);
    diag_show_line(5U, text);
    sprintf(text, "Fresh:%u", (unsigned int)air_comm_car_is_run_data_fresh());
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* Air诊断页：CRSF标准化通道0-8 */
static void diag_air_rc_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air RC");
    sprintf(text, "0:%5.0f 1:%5.0f", (double)g_air_crsf_std_ch0, (double)g_air_crsf_std_ch1);
    diag_show_line(1U, text);
    sprintf(text, "2:%5.0f 3:%5.0f", (double)g_air_crsf_std_ch2, (double)g_air_crsf_std_ch3);
    diag_show_line(2U, text);
    sprintf(text, "4:%5.0f 5:%5.0f", (double)g_air_crsf_std_ch4, (double)g_air_crsf_std_ch5);
    diag_show_line(3U, text);
    sprintf(text, "6:%5.0f 7:%5.0f", (double)g_air_crsf_std_ch6, (double)g_air_crsf_std_ch7);
    diag_show_line(4U, text);
    sprintf(text, "8:%5.0f", (double)g_air_crsf_std_ch8);
    diag_show_line(5U, text);
    sprintf(text, "State:%u", (unsigned int)(uint8)g_air_state);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* Air诊断页：Air为Car生成的规划结果 */
static void diag_air_plan_function(void)
{
    char text[32];

    diag_begin();
    diag_show_line(0U, "Air Plan");
    sprintf(text, "Valid:%u Lost:%u",
            (unsigned int)(g_air_car_plan_valid > 0.5f),
            (unsigned int)(g_air_beacon_lost_flag > 0.5f));
    diag_show_line(1U, text);
    sprintf(text, "Str:%8.3f", (double)g_air_car_plan_strafe_mps);
    diag_show_line(2U, text);
    sprintf(text, "Fwd:%8.3f", (double)g_air_car_plan_forward_mps);
    diag_show_line(3U, text);
    sprintf(text, "Cam:%u Bcn:%u",
            (unsigned int)(uint8)g_air_car_plan_camera,
            (unsigned int)(uint8)g_air_car_plan_beacon_index);
    diag_show_line(4U, text);
    sprintf(text, "Dist:%7.1fpx", (double)g_air_car_plan_dist_px);
    diag_show_line(5U, text);
    sprintf(text, "State:%u", (unsigned int)(uint8)g_air_state);
    diag_show_line(6U, text);
    diag_show_line(7U, "Back/Enter Exit");
}

/* Air诊断页：AirComm通信状态（在线/ACK/同步统计） */
static void diag_air_comm_function(void)
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
