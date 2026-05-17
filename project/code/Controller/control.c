#include "control.h"

#define CONTROL_DEG_TO_RAD (0.017453292519943295f)  // 度转弧度
#define CONTROL_PI         (3.14159265358979323846f)
#define CONTROL_TWO_PI     (6.28318530717958647692f)
#define CONTROL_YAW_RATE_AVG_10MS_SAMPLES (10U)
#define CONTROL_YAW_RATE_AVG_20MS_SAMPLES (20U)

/* 四轮PID实例（增量式，轮速控制） */
IncrementPID wheel_left_front_pid;
IncrementPID wheel_right_front_pid;
IncrementPID wheel_left_rear_pid;
IncrementPID wheel_right_rear_pid;

/* 航向控制PID实例（位置式） */
PositionalPID yaw_angle_pid;    // 角度环（外环）
PositionalPID yaw_rate_pid;     // 角速度环（内环）

/* 串级控制中间变量（供调试和诊断页读取） */
float control_yaw_angle_current = 0.0f;   // 当前航向角（rad）
float control_yaw_angle_output = 0.0f;    // 角度环输出（rad/s目标）
float control_yaw_rate_target = 0.0f;     // 角速度目标（rad/s）
float control_yaw_rate_current = 0.0f;    // 角速度环反馈（rad/s）
float control_yaw_rate_raw = 0.0f;        // 原始角速度（rad/s）
float control_yaw_rate_avg_10ms = 0.0f;   // 前10ms角速度均值（rad/s）
float control_yaw_rate_avg_20ms = 0.0f;   // 前20ms角速度均值（rad/s）
float control_yaw_rate_output = 0.0f;     // 角速度环输出（编码器计数）

/* 航向保持相关（松开旋转摇杆时锁定朝向） */
static float s_yaw_hold_target = 0.0f;    // 保持的目标航向
static uint8 s_last_rotate_active = 0U;   // 上次是否有旋转输入
static uint8 s_yaw_hold_active = 0U;      // 航向保持是否激活
static float s_yaw_rate_avg_window[CONTROL_YAW_RATE_AVG_20MS_SAMPLES];
static uint8 s_yaw_rate_avg_write_index = 0U;
static uint8 s_yaw_rate_avg_sample_count = 0U;

/* 角度归一化到±π */
static float control_normalize_angle_rad(float angle)
{
    while(angle > CONTROL_PI)
    {
        angle -= CONTROL_TWO_PI;
    }

    while(angle < -CONTROL_PI)
    {
        angle += CONTROL_TWO_PI;
    }

    return angle;
}

static void control_yaw_rate_average_reset(void)
{
    uint8 i;

    for(i = 0U; i < CONTROL_YAW_RATE_AVG_20MS_SAMPLES; i++)
    {
        s_yaw_rate_avg_window[i] = 0.0f;
    }

    s_yaw_rate_avg_write_index = 0U;
    s_yaw_rate_avg_sample_count = 0U;
    control_yaw_rate_avg_10ms = 0.0f;
    control_yaw_rate_avg_20ms = 0.0f;
}

void control_yaw_rate_average_update_1000HZ(void)
{
    uint8 i;
    uint8 index;
    uint8 count_10ms;
    uint8 count_20ms;
    float yaw_rate_radps;
    float sum_10ms;
    float sum_20ms;

    yaw_rate_radps = -g_imufilter_1000hz.gyroz * CONTROL_DEG_TO_RAD;
    s_yaw_rate_avg_window[s_yaw_rate_avg_write_index] = yaw_rate_radps;
    s_yaw_rate_avg_write_index++;
    if(s_yaw_rate_avg_write_index >= CONTROL_YAW_RATE_AVG_20MS_SAMPLES)
    {
        s_yaw_rate_avg_write_index = 0U;
    }

    if(s_yaw_rate_avg_sample_count < CONTROL_YAW_RATE_AVG_20MS_SAMPLES)
    {
        s_yaw_rate_avg_sample_count++;
    }

    count_20ms = s_yaw_rate_avg_sample_count;
    count_10ms = (count_20ms > CONTROL_YAW_RATE_AVG_10MS_SAMPLES) ?
                 CONTROL_YAW_RATE_AVG_10MS_SAMPLES :
                 count_20ms;
    sum_10ms = 0.0f;
    sum_20ms = 0.0f;
    index = s_yaw_rate_avg_write_index;

    for(i = 0U; i < count_20ms; i++)
    {
        if(0U == index)
        {
            index = CONTROL_YAW_RATE_AVG_20MS_SAMPLES;
        }
        index--;

        sum_20ms += s_yaw_rate_avg_window[index];
        if(i < count_10ms)
        {
            sum_10ms += s_yaw_rate_avg_window[index];
        }
    }

    if(count_10ms > 0U)
    {
        control_yaw_rate_avg_10ms = sum_10ms / (float)count_10ms;
    }
    if(count_20ms > 0U)
    {
        control_yaw_rate_avg_20ms = sum_20ms / (float)count_20ms;
    }
}

/* 获取当前航向角（rad，±π）
 * 取反是因为IMU安装方向
 */
float control_get_current_yaw_angle(void)
{
    return control_normalize_angle_rad(-g_euler.yaw * CONTROL_DEG_TO_RAD);
}

/* 重置航向保持：锁定当前航向 */
void control_yaw_hold_reset(void)
{
    s_last_rotate_active = 0U;
    s_yaw_hold_active = 0U;
    s_yaw_hold_target = control_get_current_yaw_angle();
}

/* 初始化单个轮速PID（从menu_config读取参数） */
static void control_speed_pid_init(IncrementPID *pid)
{
    IncrementPID_Init(pid, wheel_kp, wheel_ki, wheel_kd, wheel_output_limit);
}

/* 实时更新轮速PID参数（菜单修改后立即生效） */
static void control_speed_pid_apply_params(IncrementPID *pid)
{
    pid->kp = wheel_kp;
    pid->ki = wheel_ki;
    pid->kd = wheel_kd;
    pid->output_limit = wheel_output_limit;
}

/* 初始化角速度环PID */
static void control_yaw_rate_pid_init(void)
{
    PositionalPID_Init(&yaw_rate_pid, 0.0f, yaw_rate_kp, yaw_rate_ki,
                       yaw_rate_kd, yaw_rate_i_limit, yaw_rate_output_limit);
}

/* 初始化角度环PID */
static void control_yaw_angle_pid_init(void)
{
    PositionalPID_Init(&yaw_angle_pid, 0.0f, yaw_angle_kp, yaw_angle_ki,
                       yaw_angle_kd, yaw_angle_i_limit, yaw_angle_output_limit);
}

/* 实时更新角速度环PID参数 */
static void control_yaw_rate_pid_apply_params(void)
{
    yaw_rate_pid.kp_2 = 0.0f;
    yaw_rate_pid.kp_1 = yaw_rate_kp;
    yaw_rate_pid.ki = yaw_rate_ki;
    yaw_rate_pid.kd = yaw_rate_kd;
    yaw_rate_pid.i_limit = yaw_rate_i_limit;
    yaw_rate_pid.output_limit = yaw_rate_output_limit;
}

/* 实时更新角度环PID参数 */
static void control_yaw_angle_pid_apply_params(void)
{
    yaw_angle_pid.kp_2 = 0.0f;
    yaw_angle_pid.kp_1 = yaw_angle_kp;
    yaw_angle_pid.ki = yaw_angle_ki;
    yaw_angle_pid.kd = yaw_angle_kd;
    yaw_angle_pid.i_limit = yaw_angle_i_limit;
    yaw_angle_pid.output_limit = yaw_angle_output_limit;
}

/* 初始化速度环（调用reset） */
void control_speed_loop_init(void)
{
    control_speed_loop_reset();
}

/* 初始化串级控制：角度环 + 角速度环 + 速度环 */
void control_cascade_init(void)
{
    control_yaw_rate_average_reset();
    control_yaw_angle_pid_init();
    control_yaw_rate_pid_init();
    control_speed_loop_init();
}

/* 复位四轮速度环PID */
void control_speed_loop_reset(void)
{
    control_speed_pid_init(&wheel_left_front_pid);
    control_speed_pid_init(&wheel_right_front_pid);
    control_speed_pid_init(&wheel_left_rear_pid);
    control_speed_pid_init(&wheel_right_rear_pid);
}

/* 复位角度环：清零PID + 清零输出和目标 */
void control_yaw_angle_loop_reset(void)
{
    control_yaw_angle_pid_init();
    control_yaw_angle_output = 0.0f;
    control_yaw_rate_target = 0.0f;
}

/* 复位整个串级控制：所有PID + 中间变量 + 航向保持 */
void control_cascade_reset(void)
{
    control_yaw_angle_pid_init();
    control_yaw_rate_pid_init();
    control_speed_loop_reset();
    control_yaw_angle_current = 0.0f;
    control_yaw_angle_output = 0.0f;
    control_yaw_rate_target = 0.0f;
    control_yaw_rate_current = 0.0f;
    control_yaw_rate_raw = 0.0f;
    control_yaw_rate_output = 0.0f;
    control_yaw_hold_reset();
}

/* 角度环更新（25HZ）
 * 输入：目标航向角（rad）
 * 输出：角速度目标（rad/s）
 * 处理：归一化角度→算误差→PID→输出角速度目标
 */
float control_yaw_angle_loop_update_25HZ(float yaw_angle_target)
{
    float yaw_error;

    control_yaw_angle_pid_apply_params();

    control_yaw_angle_current = control_get_current_yaw_angle();
    yaw_angle_target = control_normalize_angle_rad(yaw_angle_target);
    yaw_error = control_normalize_angle_rad(yaw_angle_target - control_yaw_angle_current);

    control_yaw_angle_output = PositionalPID_Update(&yaw_angle_pid, yaw_error, 0.0f);
    control_yaw_rate_target = control_yaw_angle_output;
    return control_yaw_rate_target;
}

/* 角速度环更新（50HZ）
 * 输入：角速度目标（rad/s，来自角度环或遥控器直接指定）
 * 输出：编码器计数（给速度环作为旋转分量）
 * 数据来源：1kHz更新的前20ms角速度均值
 */
float control_yaw_rate_loop_update_50HZ(float yaw_rate_target)
{
    control_yaw_rate_pid_apply_params();

    control_yaw_rate_target = yaw_rate_target;
    control_yaw_rate_raw = -ICM42688.gyro_z * CONTROL_DEG_TO_RAD;
    control_yaw_rate_current = control_yaw_rate_avg_20ms;
    control_yaw_rate_output = PositionalPID_Update(&yaw_rate_pid,
                                                   yaw_rate_target,
                                                   control_yaw_rate_current);
    return control_yaw_rate_output;
}

/* 使用当前角速度环输出更新速度环（100HZ）
 * forward/strafe来自car_forward_target/car_strafe_target
 * rotate使用全局control_yaw_rate_output
 */
void control_cascade_speed_loop_update_100HZ(float forward_target, float strafe_target)
{
    control_cascade_speed_loop_update_with_rotate_100HZ(forward_target, strafe_target, control_yaw_rate_output);
}

/* 航向保持更新（25HZ）
 * rotate_target非零：更新保持目标为当前航向，不激活保持
 * rotate_target为零：如果刚松开旋转或保持未激活，重新锁航向；然后持续执行角度环保持
 * 用途：Mode0遥控模式，松开旋转摇杆后自动锁定朝向
 */
void control_yaw_hold_update_25HZ(float rotate_target)
{
    if(0.0f != rotate_target)
    {
        s_yaw_hold_target = control_get_current_yaw_angle();
        s_yaw_hold_active = 0U;
    }
    else
    {
        if((0U == s_yaw_hold_active) || (0U != s_last_rotate_active))
        {
            s_yaw_hold_target = control_get_current_yaw_angle();
            control_yaw_angle_loop_reset();
            s_yaw_hold_active = 1U;
        }

        control_yaw_angle_loop_update_25HZ(s_yaw_hold_target);
    }

    s_last_rotate_active = (0.0f != rotate_target) ? 1U : 0U;
}

/* 麦克纳姆轮运动学解算 + 速度环更新（100HZ）
 * 前+右+旋=各轮目标速度
 * 注意：这是经典麦克纳姆轮45度排列的解算公式
 */
void control_cascade_speed_loop_update_with_rotate_100HZ(float forward_target, float strafe_target, float rotate_target)
{
    float left_front_target = forward_target - strafe_target - rotate_target;
    float right_front_target = forward_target + strafe_target + rotate_target;
    float left_rear_target = forward_target + strafe_target - rotate_target;
    float right_rear_target = forward_target - strafe_target + rotate_target;

    control_speed_loop_update_100HZ(left_front_target, right_front_target,
                                    left_rear_target, right_rear_target);
}

/* 串级控制完整更新（50HZ）
 * 注意：实际在car_loop中，角速度环50HZ跑，速度环100HZ跑，这里只是一种组合调用
 */
void control_cascade_update_50HZ(float forward_target, float strafe_target, float yaw_rate_target)
{
    control_yaw_rate_loop_update_50HZ(yaw_rate_target);
    control_cascade_speed_loop_update_100HZ(forward_target, strafe_target);
}

/* 四轮速度环更新（100HZ，最核心的控制函数）
 * 输入：四轮目标速度（编码器计数）
 * 过程：实时应用PID参数 → 四轮增量式PID → 输出PWM
 * 注意：每周期都从menu_config读取参数，支持菜单实时调参
 */
void control_speed_loop_update_100HZ(float left_front_target, float right_front_target,
                                     float left_rear_target, float right_rear_target)
{
    control_speed_pid_apply_params(&wheel_left_front_pid);
    control_speed_pid_apply_params(&wheel_right_front_pid);
    control_speed_pid_apply_params(&wheel_left_rear_pid);
    control_speed_pid_apply_params(&wheel_right_rear_pid);

    float left_front_pwm = IncrementPID_Update(&wheel_left_front_pid, left_front_target,
                                               encoder_get_left_front_filtered_count());
    float right_front_pwm = IncrementPID_Update(&wheel_right_front_pid, right_front_target,
                                                encoder_get_right_front_filtered_count());
    float left_rear_pwm = IncrementPID_Update(&wheel_left_rear_pid, left_rear_target,
                                              encoder_get_left_rear_filtered_count());
    float right_rear_pwm = IncrementPID_Update(&wheel_right_rear_pid, right_rear_target,
                                               encoder_get_right_rear_filtered_count());

    mecanum_motor_set_all((int16_t)left_front_pwm,
                          (int16_t)right_front_pwm,
                          (int16_t)left_rear_pwm,
                          (int16_t)right_rear_pwm);
}

/* 停止速度环：复位PID + 立即停电机（安全停机） */
void control_speed_loop_stop(void)
{
    control_speed_loop_reset();
    mecanum_motor_stop();
}

/* 停止串级控制：复位所有PID + 停电机（紧急停机入口） */
void control_cascade_stop(void)
{
    control_cascade_reset();
    mecanum_motor_stop();
}
