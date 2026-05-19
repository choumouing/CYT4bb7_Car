/* Mode2：目标点导航模式
 *
 * 功能：UWB+里程计多点巡航，自动在目标点间切换
 * 状态机：IDLE → FOLLOW_TAG（跟随标签）/ GOTO_TARGET（前往目标）/ TARGET_REACHED（到达）
 * 切换逻辑：标签在线且车在保护半径内 → 找候选目标 → 前往 → 到达标记 → 找下一个
 *          标签不在线或车超出保护半径 → 退回FOLLOW_TAG（Mode1跟随）
 * 数据来源：里程计g_odometer + UWB ALX_AOA + IMU航向角
 * 输出：car_forward/strafe_target（编码器计数），yaw由控制层锁0
 */
#include "car_mode.h"


car_mode2_state_t g_car_mode2_state = {0};  // 运行状态（供诊断页读取）

static PositionalPID s_target_forward_pid;   // 目标导航前后轴PID
static PositionalPID s_target_strafe_pid;    // 目标导航左右轴PID

/* 计算两点间欧氏距离（m） */
static float car_mode2_distance(float strafe_a, float forward_a,
                                    float strafe_b, float forward_b)
{
    float ds;
    float df;

    ds = strafe_a - strafe_b;
    df = forward_a - forward_b;
    return sqrtf((ds * ds) + (df * df));
}

/* 初始化/重置目标导航PID（使用car_mode.h中的常量参数） */
static void car_mode2_pid_init(void)
{
    PositionalPID_Init(&s_target_forward_pid,
                       0.0f,
                       TARGET_FOLLOW_POS_KP,
                       TARGET_FOLLOW_POS_KI,
                       TARGET_FOLLOW_POS_KD,
                       TARGET_FOLLOW_POS_I_LIMIT,
                       TARGET_FOLLOW_OUTPUT_LIMIT);
    PositionalPID_Init(&s_target_strafe_pid,
                       0.0f,
                       TARGET_FOLLOW_POS_KP,
                       TARGET_FOLLOW_POS_KI,
                       TARGET_FOLLOW_POS_KD,
                       TARGET_FOLLOW_POS_I_LIMIT,
                       TARGET_FOLLOW_OUTPUT_LIMIT);
}

/* 清零所有运行时状态（坐标、输出、子状态等） */
static void car_mode2_clear_runtime(void)
{
    g_car_mode2_state.car_strafe_m = 0.0f;
    g_car_mode2_state.car_forward_m = 0.0f;
    g_car_mode2_state.tag_strafe_m = 0.0f;
    g_car_mode2_state.tag_forward_m = 0.0f;
    g_car_mode2_state.tag_relative_strafe_m = 0.0f;
    g_car_mode2_state.tag_relative_forward_m = 0.0f;
    g_car_mode2_state.car_tag_distance_m = 0.0f;
    g_car_mode2_state.target_tag_distance_m = 0.0f;
    g_car_mode2_state.target_car_distance_m = 0.0f;
    g_car_mode2_state.target_error_strafe_m = 0.0f;
    g_car_mode2_state.target_error_forward_m = 0.0f;
    g_car_mode2_state.target_pid_strafe_output = 0.0f;
    g_car_mode2_state.target_pid_forward_output = 0.0f;
    g_car_mode2_state.forward_target = 0.0f;
    g_car_mode2_state.strafe_target = 0.0f;
    g_car_mode2_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
    g_car_mode2_state.candidate_index = TARGET_FOLLOW_INVALID_INDEX;
    g_car_mode2_state.mode = TARGET_FOLLOW_MODE_IDLE;
    g_car_mode2_state.tag_online = 0U;
    g_car_mode2_state.car_in_tag_range = 0U;
    g_car_mode2_state.target_in_tag_range = 0U;
    g_car_mode2_state.output_valid = 0U;
}

/* 向量限幅：等比例缩小使最大分量不超过TARGET_FOLLOW_OUTPUT_LIMIT
 * 防止单轴过大导致麦克纳姆轮解算异常
 */
static void car_mode2_apply_vector_limit(void)
{
    float abs_forward;
    float abs_strafe;
    float max_value;
    float scale;

    abs_forward = car_math_absf(g_car_mode2_state.forward_target);
    abs_strafe = car_math_absf(g_car_mode2_state.strafe_target);
    max_value = (abs_forward > abs_strafe) ? abs_forward : abs_strafe;

    if(max_value <= TARGET_FOLLOW_OUTPUT_LIMIT)
    {
        return;
    }

    scale = TARGET_FOLLOW_OUTPUT_LIMIT / max_value;
    g_car_mode2_state.forward_target *= scale;
    g_car_mode2_state.strafe_target *= scale;
}

/* 复制Mode1的跟随输出到Mode2（当车不在标签保护半径内或无候选目标时降级到跟随） */
static void car_mode2_copy_tag_follow_output(void)
{
    g_car_mode2_state.mode = TARGET_FOLLOW_MODE_FOLLOW_TAG;
    g_car_mode2_state.output_valid = g_car_mode1_state.output_valid;
    g_car_mode2_state.target_pid_forward_output = 0.0f;
    g_car_mode2_state.target_pid_strafe_output = 0.0f;
    g_car_mode2_state.forward_target = (0U != g_car_mode1_state.output_valid) ?
                                           g_car_mode1_state.forward_target : 0.0f;
    g_car_mode2_state.strafe_target = (0U != g_car_mode1_state.output_valid) ?
                                          g_car_mode1_state.strafe_target : 0.0f;
}

/* 判断目标点是否合格（未到达 + 在标签保护半径内 + 在匹配半径内）
 * 返回1=合格，0=不合格
 * distance_m：可选输出，目标到标签的距离
 */
static uint8 car_mode2_target_is_eligible(uint8 index, float *distance_m)
{
    float distance;

    if((index >= TARGET_FOLLOW_MAX_TARGETS) ||
       (0U == g_car_mode2_state.targets[index].valid) ||
       (0U != g_car_mode2_state.targets[index].reached))
    {
        return 0U;
    }

    distance = car_mode2_distance(g_car_mode2_state.targets[index].strafe_m,
                                      g_car_mode2_state.targets[index].forward_m,
                                      g_car_mode2_state.tag_strafe_m,
                                      g_car_mode2_state.tag_forward_m);
    if(0 != distance_m)
    {
        *distance_m = distance;
    }

    if((distance <= TARGET_FOLLOW_TARGET_MATCH_RADIUS_M) &&
       (distance <= TARGET_FOLLOW_TAG_GUARD_RADIUS_M))
    {
        return 1U;
    }

    return 0U;
}

/* 找最近的合格目标点
 * 优先返回当前活跃目标（如果仍合格），否则遍历所有目标找最近的
 * 返回目标索引，无合格目标返回TARGET_FOLLOW_INVALID_INDEX
 */
static uint8 car_mode2_find_candidate(float *candidate_distance_m)
{
    uint8 i;
    uint8 best_index;
    float best_distance;
    float distance;

    best_index = TARGET_FOLLOW_INVALID_INDEX;
    best_distance = 0.0f;

    if((g_car_mode2_state.active_index != TARGET_FOLLOW_INVALID_INDEX) &&
       (0U != car_mode2_target_is_eligible(g_car_mode2_state.active_index, &distance)))
    {
        if(0 != candidate_distance_m)
        {
            *candidate_distance_m = distance;
        }
        return g_car_mode2_state.active_index;
    }

    for(i = 0U; i < g_car_mode2_state.target_count; i++)
    {
        if(0U == car_mode2_target_is_eligible(i, &distance))
        {
            continue;
        }

        if((TARGET_FOLLOW_INVALID_INDEX == best_index) || (distance < best_distance))
        {
            best_index = i;
            best_distance = distance;
        }
    }

    if((TARGET_FOLLOW_INVALID_INDEX != best_index) && (0 != candidate_distance_m))
    {
        *candidate_distance_m = best_distance;
    }

    return best_index;
}

/* 刷新标签全局坐标
 * 输入：UWB滤波后相对坐标（cm）+ 当前航向角 + 里程计
 * 计算：将UWB相对坐标旋转到全局坐标系，加上车当前位置
 * 输出：tag_strafe_m/tag_forward_m（标签全局坐标），car_in_tag_range（车是否在保护半径内）
 */
static void car_mode2_refresh_tag_position(void)
{
    float rel_x_cm;
    float rel_y_cm;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float rel_strafe_m;
    float rel_forward_m;

    rel_x_cm = 0.0f;
    rel_y_cm = 0.0f;
    (void)ALX_AOA_GetFilteredXY(&rel_x_cm, &rel_y_cm);

    rel_strafe_m = -rel_x_cm * 0.01f;
    rel_forward_m = rel_y_cm * 0.01f;
    yaw_rad = Control_GetYawAngle();
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    g_car_mode2_state.car_strafe_m = g_odometer.strafe_distance;
    g_car_mode2_state.car_forward_m = g_odometer.forward_distance;
    g_car_mode2_state.tag_relative_strafe_m = rel_strafe_m;
    g_car_mode2_state.tag_relative_forward_m = rel_forward_m;
    g_car_mode2_state.tag_forward_m =
        g_car_mode2_state.car_forward_m +
        ((cos_yaw * rel_forward_m) - (sin_yaw * rel_strafe_m));
    g_car_mode2_state.tag_strafe_m =
        g_car_mode2_state.car_strafe_m +
        ((sin_yaw * rel_forward_m) + (cos_yaw * rel_strafe_m));
    g_car_mode2_state.car_tag_distance_m =
        sqrtf((rel_strafe_m * rel_strafe_m) + (rel_forward_m * rel_forward_m));
    g_car_mode2_state.car_in_tag_range =
        (g_car_mode2_state.car_tag_distance_m <= TARGET_FOLLOW_TAG_GUARD_RADIUS_M) ? 1U : 0U;
}

/* 前往指定目标点
 * 流程：
 *   1. 计算全局误差 → 到达判定（<REACHED_RADIUS则标记到达并返回）
 *   2. 全局误差旋转到车体坐标系（用航向角）
 *   3. 死区处理 → PID计算 → 向量限幅 → 写输出
 * 输出：forward_target + strafe_target（编码器计数）
 */
static void car_mode2_drive_to_target(uint8 index)
{
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float error_strafe_global;
    float error_forward_global;
    float error_strafe_body;
    float error_forward_body;

    error_strafe_global =
        g_car_mode2_state.targets[index].strafe_m - g_car_mode2_state.car_strafe_m;
    error_forward_global =
        g_car_mode2_state.targets[index].forward_m - g_car_mode2_state.car_forward_m;
    g_car_mode2_state.target_error_strafe_m = error_strafe_global;
    g_car_mode2_state.target_error_forward_m = error_forward_global;
    g_car_mode2_state.target_car_distance_m =
        sqrtf((error_strafe_global * error_strafe_global) +
              (error_forward_global * error_forward_global));

    if(g_car_mode2_state.target_car_distance_m <= TARGET_FOLLOW_REACHED_RADIUS_M)
    {
        g_car_mode2_state.targets[index].reached = 1U;
        g_car_mode2_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
        g_car_mode2_state.mode = TARGET_FOLLOW_MODE_TARGET_REACHED;
        g_car_mode2_state.forward_target = 0.0f;
        g_car_mode2_state.strafe_target = 0.0f;
        g_car_mode2_state.target_pid_forward_output = 0.0f;
        g_car_mode2_state.target_pid_strafe_output = 0.0f;
        g_car_mode2_state.output_valid = 1U;
        car_mode2_pid_init();
        return;
    }

    yaw_rad = Control_GetYawAngle();
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    error_forward_body = (cos_yaw * error_forward_global) + (sin_yaw * error_strafe_global);
    error_strafe_body = (-sin_yaw * error_forward_global) + (cos_yaw * error_strafe_global);
    error_forward_body = car_math_deadband(error_forward_body, TARGET_FOLLOW_POSITION_DEADBAND_M);
    error_strafe_body = car_math_deadband(error_strafe_body, TARGET_FOLLOW_POSITION_DEADBAND_M);

    g_car_mode2_state.forward_target =
        car_math_limit_absf(PositionalPID_Update(&s_target_forward_pid, error_forward_body, 0.0f),
                            TARGET_FOLLOW_OUTPUT_LIMIT);
    g_car_mode2_state.strafe_target =
        car_math_limit_absf(PositionalPID_Update(&s_target_strafe_pid, error_strafe_body, 0.0f),
                            TARGET_FOLLOW_OUTPUT_LIMIT);
    g_car_mode2_state.target_pid_forward_output = g_car_mode2_state.forward_target;
    g_car_mode2_state.target_pid_strafe_output = g_car_mode2_state.strafe_target;
    car_mode2_apply_vector_limit();
    g_car_mode2_state.target_pid_forward_output = g_car_mode2_state.forward_target;
    g_car_mode2_state.target_pid_strafe_output = g_car_mode2_state.strafe_target;
    g_car_mode2_state.mode = TARGET_FOLLOW_MODE_GOTO_TARGET;
    g_car_mode2_state.output_valid = 1U;
}

void car_mode2_init(void)
{
    car_mode2_clear_targets();
    car_mode2_reset();
}

void car_mode2_reset(void)
{
    car_mode2_pid_init();
    car_mode2_clear_runtime();
}

void car_mode2_restart_targets(void)
{
    uint8 i;

    for(i = 0U; i < TARGET_FOLLOW_MAX_TARGETS; i++)
    {
        g_car_mode2_state.targets[i].reached = 0U;
    }

    car_mode2_reset();
}

void car_mode2_clear_targets(void)
{
    uint8 i;

    for(i = 0U; i < TARGET_FOLLOW_MAX_TARGETS; i++)
    {
        g_car_mode2_state.targets[i].strafe_m = 0.0f;
        g_car_mode2_state.targets[i].forward_m = 0.0f;
        g_car_mode2_state.targets[i].reached = 0U;
        g_car_mode2_state.targets[i].valid = 0U;
    }

    g_car_mode2_state.target_count = 0U;
    g_car_mode2_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
    g_car_mode2_state.candidate_index = TARGET_FOLLOW_INVALID_INDEX;
}

uint8 car_mode2_add_target(float strafe_m, float forward_m)
{
    if(g_car_mode2_state.target_count >= TARGET_FOLLOW_MAX_TARGETS)
    {
        return 0U;
    }

    return car_mode2_set_target(g_car_mode2_state.target_count, strafe_m, forward_m);
}

uint8 car_mode2_set_target(uint8 index, float strafe_m, float forward_m)
{
    if(index >= TARGET_FOLLOW_MAX_TARGETS)
    {
        return 0U;
    }

    g_car_mode2_state.targets[index].strafe_m = strafe_m;
    g_car_mode2_state.targets[index].forward_m = forward_m;
    g_car_mode2_state.targets[index].reached = 0U;
    g_car_mode2_state.targets[index].valid = 1U;

    if(index >= g_car_mode2_state.target_count)
    {
        g_car_mode2_state.target_count = (uint8)(index + 1U);
    }

    return 1U;
}

/* Mode2主更新函数（25HZ）
 * 数据流：
 *   1. 先调用Mode1更新（计算跟随输出备用）
 *   2. 检查标签在线 → 不在线则清零返回
 *   3. 刷新标签全局坐标
 *   4. 车不在标签保护半径内 → 降级到跟随模式
 *   5. 找候选目标 → 无合格目标 → 降级到跟随
 *   6. 有候选目标 → 前往目标 → 写输出
 * 最终输出：car_forward/strafe_target，yaw由控制层锁0
 */
void car_mode2_update_25HZ(uint32 now_ms)
{
    float tag_target_distance;
    float rel_x_cm;
    float rel_y_cm;
    uint8 candidate;

    tag_target_distance = 0.0f;
    rel_x_cm = 0.0f;
    rel_y_cm = 0.0f;
    g_car_mode2_state.forward_target = 0.0f;
    g_car_mode2_state.strafe_target = 0.0f;
    g_car_mode2_state.target_pid_forward_output = 0.0f;
    g_car_mode2_state.target_pid_strafe_output = 0.0f;
    g_car_mode2_state.output_valid = 0U;
    g_car_mode2_state.target_in_tag_range = 0U;
    g_car_mode2_state.candidate_index = TARGET_FOLLOW_INVALID_INDEX;

    car_mode1_update_25HZ(now_ms);
    g_car_mode2_state.tag_online = ALX_AOA_IsTagOnline(now_ms, TARGET_FOLLOW_UWB_TIMEOUT_MS);
    if((0U == g_car_mode2_state.tag_online) ||
       (0U == ALX_AOA_GetFilteredXY(&rel_x_cm, &rel_y_cm)))
    {
        car_mode2_pid_init();
        g_car_mode2_state.mode = TARGET_FOLLOW_MODE_IDLE;
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        return;
    }

    car_mode2_refresh_tag_position();
    if(0U == g_car_mode2_state.car_in_tag_range)
    {
        g_car_mode2_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
        car_mode2_pid_init();
        car_mode2_copy_tag_follow_output();
        car_forward_target = g_car_mode2_state.forward_target;
        car_strafe_target = g_car_mode2_state.strafe_target;
        return;
    }

    candidate = car_mode2_find_candidate(&tag_target_distance);
    g_car_mode2_state.candidate_index = candidate;
    g_car_mode2_state.target_tag_distance_m = tag_target_distance;

    if(TARGET_FOLLOW_INVALID_INDEX == candidate)
    {
        g_car_mode2_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
        car_mode2_pid_init();
        car_mode2_copy_tag_follow_output();
        car_forward_target = g_car_mode2_state.forward_target;
        car_strafe_target = g_car_mode2_state.strafe_target;
        return;
    }

    if(g_car_mode2_state.active_index != candidate)
    {
        car_mode2_pid_init();
    }

    g_car_mode2_state.active_index = candidate;
    g_car_mode2_state.target_in_tag_range = 1U;
    car_mode2_drive_to_target(candidate);
    car_forward_target = (0U != g_car_mode2_state.output_valid) ? g_car_mode2_state.forward_target : 0.0f;
    car_strafe_target = (0U != g_car_mode2_state.output_valid) ? g_car_mode2_state.strafe_target : 0.0f;
}
