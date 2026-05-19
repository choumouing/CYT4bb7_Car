/* Mode1：UWB跟随模式
 * 功能：跟随UWB标签移动
 * 数据流：UWB → 滤波 → 死区 → PID → 输出（编码器计数）
 * 调用频率：25HZ
 * 注意：模式层不输出旋转目标，yaw由控制层锁0
 */
#include "car_mode.h"


car_mode1_state_t g_car_mode1_state = {0};  // 运行状态（供诊断页读取）

static PositionalPID s_car_mode1_x_pid;     // X轴位置环PID
static PositionalPID s_car_mode1_y_pid;     // Y轴位置环PID

/* 向量限幅：等比例缩小使最大分量不超过limit
 * 防止单轴过大导致麦克纳姆轮解算异常
 */
static void car_mode1_apply_vector_limit(void)
{
    float abs_forward;
    float abs_strafe;
    float max_value;
    float scale;

    abs_forward = car_math_absf(g_car_mode1_state.forward_target);
    abs_strafe = car_math_absf(g_car_mode1_state.strafe_target);
    max_value = (abs_forward > abs_strafe) ? abs_forward : abs_strafe;

    if(max_value <= uwb_follow_output_limit)
    {
        return;
    }

    scale = uwb_follow_output_limit / max_value;
    g_car_mode1_state.forward_target *= scale;
    g_car_mode1_state.strafe_target *= scale;
}

/* 初始化X/Y轴位置环PID */
static void car_mode1_pid_init(void)
{
    PositionalPID_Init(&s_car_mode1_x_pid,
                       0.0f,
                       uwb_follow_x_kp,
                       uwb_follow_x_ki,
                       uwb_follow_x_kd,
                       uwb_follow_i_limit,
                       uwb_follow_output_limit);
    PositionalPID_Init(&s_car_mode1_y_pid,
                       0.0f,
                       uwb_follow_y_kp,
                       uwb_follow_y_ki,
                       uwb_follow_y_kd,
                       uwb_follow_i_limit,
                       uwb_follow_output_limit);
}

/* 实时更新PID参数（支持菜单调参） */
static void car_mode1_pid_apply_params(void)
{
    s_car_mode1_x_pid.kp_2 = 0.0f;
    s_car_mode1_x_pid.kp_1 = uwb_follow_x_kp;
    s_car_mode1_x_pid.ki = uwb_follow_x_ki;
    s_car_mode1_x_pid.kd = uwb_follow_x_kd;
    s_car_mode1_x_pid.i_limit = uwb_follow_i_limit;
    s_car_mode1_x_pid.output_limit = uwb_follow_output_limit;

    s_car_mode1_y_pid.kp_2 = 0.0f;
    s_car_mode1_y_pid.kp_1 = uwb_follow_y_kp;
    s_car_mode1_y_pid.ki = uwb_follow_y_ki;
    s_car_mode1_y_pid.kd = uwb_follow_y_kd;
    s_car_mode1_y_pid.i_limit = uwb_follow_i_limit;
    s_car_mode1_y_pid.output_limit = uwb_follow_output_limit;
}

void car_mode1_init(void)
{
    car_mode1_reset();
}

/* 重置：清零PID + 清零所有状态 */
void car_mode1_reset(void)
{
    car_mode1_pid_init();

    g_car_mode1_state.error_x_cm = 0.0f;
    g_car_mode1_state.error_y_cm = 0.0f;
    g_car_mode1_state.x_pid_p_term = 0.0f;
    g_car_mode1_state.x_pid_i_term = 0.0f;
    g_car_mode1_state.x_pid_d_term = 0.0f;
    g_car_mode1_state.y_pid_p_term = 0.0f;
    g_car_mode1_state.y_pid_i_term = 0.0f;
    g_car_mode1_state.y_pid_d_term = 0.0f;
    g_car_mode1_state.raw_x_cm = 0.0f;
    g_car_mode1_state.raw_y_cm = 0.0f;
    g_car_mode1_state.filt_x_cm = 0.0f;
    g_car_mode1_state.filt_y_cm = 0.0f;
    g_car_mode1_state.forward_target = 0.0f;
    g_car_mode1_state.strafe_target = 0.0f;
    g_car_mode1_state.tag_online = 0U;
    g_car_mode1_state.output_valid = 0U;
}

/* UWB跟随更新（25HZ）
 * 数据流：
 *   1. 检查标签在线（超时判断）
 *   2. 读取原始和滤波后的UWB坐标
 *   3. 死区处理消除小误差
 *   4. PID计算（以(0,0)为目标）
 *   5. 限幅 + 向量归一化
 *   6. 写入全局car_forward/strafe_target
 * 注意：X轴取反（UWB坐标系与车体坐标系可能相反）
 */
void car_mode1_update_25HZ(uint32 now_ms)
{
    ALX_AOA_Position_t position;

    g_car_mode1_state.output_valid = 0U;
    g_car_mode1_state.tag_online = ALX_AOA_IsTagOnline(now_ms, UWB_FOLLOW_TIMEOUT_MS);

    /* 标签不在线或数据无效：清零输出（安全） */
    if((0U == g_car_mode1_state.tag_online) ||
       (0U == ALX_AOA_GetLatest(&position)) ||
       (0U == ALX_AOA_GetFilteredXY(&g_car_mode1_state.filt_x_cm,
                                    &g_car_mode1_state.filt_y_cm)))
    {
        car_mode1_reset();
        return;
    }

    car_mode1_pid_apply_params();
    g_car_mode1_state.raw_x_cm = (float)position.x_cm;
    g_car_mode1_state.raw_y_cm = (float)position.y_cm;

    /* 死区处理 */
    g_car_mode1_state.error_x_cm =
        car_math_deadband(g_car_mode1_state.filt_x_cm, uwb_follow_deadband_x_cm);
    g_car_mode1_state.error_y_cm =
        car_math_deadband(g_car_mode1_state.filt_y_cm, uwb_follow_deadband_y_cm);

    /* PID计算（目标为0） + 输出限幅 */
    g_car_mode1_state.strafe_target =
        -car_math_limit_absf(PositionalPID_Update(&s_car_mode1_x_pid,
                                                  g_car_mode1_state.error_x_cm,
                                                  0.0f),
                             uwb_follow_output_limit);
    g_car_mode1_state.forward_target =
        car_math_limit_absf(PositionalPID_Update(&s_car_mode1_y_pid,
                                                 g_car_mode1_state.error_y_cm,
                                                 0.0f),
                            uwb_follow_output_limit);

    /* 保存PID中间值（调试用） */
    g_car_mode1_state.x_pid_p_term = s_car_mode1_x_pid.p_term;
    g_car_mode1_state.x_pid_i_term = s_car_mode1_x_pid.i_term;
    g_car_mode1_state.x_pid_d_term = s_car_mode1_x_pid.d_term;
    g_car_mode1_state.y_pid_p_term = s_car_mode1_y_pid.p_term;
    g_car_mode1_state.y_pid_i_term = s_car_mode1_y_pid.i_term;
    g_car_mode1_state.y_pid_d_term = s_car_mode1_y_pid.d_term;

    car_mode1_apply_vector_limit();
    g_car_mode1_state.output_valid = 1U;

    /* 写入全局目标 */
    car_forward_target = g_car_mode1_state.forward_target;
    car_strafe_target = g_car_mode1_state.strafe_target;
}
