#include "odometer.h"


/* ===== 基本参数 ===== */
#define ODOMETER_UPDATE_DT_S                (0.01f)     /* 100Hz 调用周期 */
#define ODOMETER_STARTUP_HOLD_TICKS         (50U)       /* 启动稳定期，约 0.5s */
#define ODOMETER_FORWARD_COUNT_PER_METER    (11287.0f)  /* 前后轴编码器脉冲/米 */
#define ODOMETER_STRAFE_COUNT_PER_METER_ABS (12100.0f)  /* 横向轴编码器脉冲/米(绝对值) */

/* ===== 速度衰减模型系数（离线标定值，别手改） ===== */
#define ODOMETER_KX_SPEED                   (0.00529850743f)  /* 前向：速度风险权重 */
#define ODOMETER_KY_SPEED                   (0.0f)            /* 横向：速度风险权重 */
#define ODOMETER_KX_RESIDUAL                (0.488792866f)    /* 前向：残差风险权重 */
#define ODOMETER_KY_RESIDUAL                (0.00697509527f)  /* 横向：残差风险权重 */
#define ODOMETER_KX_REVERSE                 (1.00619317f)     /* 前向：反转惩罚权重 */
#define ODOMETER_KY_REVERSE                 (0.00000105576f)  /* 横向：反转惩罚权重 */
#define ODOMETER_KX_YAW                     (0.68f)           /* 前向：偏航风险权重 */
#define ODOMETER_KY_YAW                     (0.80f)           /* 横向：偏航风险权重 */
#define ODOMETER_K_DUAL_AXIS                (0.0653664524f)   /* 双轴同时运动交叉干扰 */
#define ODOMETER_V_REF_MPS                  (1.66942520f)     /* 速度归一化参考值 */
#define ODOMETER_RESIDUAL_REF_MPS2          (8.33671585f)     /* 加速度残差归一化参考 */
#define ODOMETER_RISK_GAMMA                 (7.68239773f)     /* 残差风险曲线指数 */
#define ODOMETER_YAW_GAIN                   (0.72f)           /* 偏航角投影增益（防过转） */
#define ODOMETER_DEAD_FORWARD_MPS           (0.000000424f)    /* 前向死区 */
#define ODOMETER_DEAD_STRAFE_MPS            (0.00496915618f)  /* 横向死区 */
#define ODOMETER_SPEED_POWER_X              (2.51620747f)     /* 前向速度风险曲线幂次 */
#define ODOMETER_SPEED_POWER_Y              (1.26317978f)     /* 横向速度风险曲线幂次 */
#define ODOMETER_CROSS_AXIS                 (1.83240615f)     /* 交叉轴耦合系数 */
#define ODOMETER_REVERSE_REF_MPS            (0.0759208411f)   /* 反转归一化参考 */
#define ODOMETER_YAW_RATE_REF_RPS           (0.936060514f)    /* 偏航角速率归一化参考 */

/* ===== 融合滤波参数 ===== */
#define ODOMETER_ALPHA_MAX                  (0.03f)     /* 编码器 vs 加速度计融合上限 */
#define ODOMETER_ALPHA_TAU_S                (0.06f)     /* alpha 低通时间常数 */

/* ===== 数学常量（注意：DEG_TO_RAD / PI 等已有 car_math 定义，
 *    这里保留是为了避免破坏已标定的浮点精度，改用 car_math 时需逐个验证） ===== */
#define ODOMETER_DEG_TO_RAD                 (0.017453292519943295f)
#define ODOMETER_RAD_TO_DEG                 (57.295779513082320876f)
#define ODOMETER_PI                         (3.14159265358979323846f)
#define ODOMETER_TWO_PI                     (6.28318530717958647692f)
#define ODOMETER_EPSILON                    (1.0e-6f)   /* 防除零 */

/* ===== 加速度偏置学习条件 ===== */
#define ODOMETER_ACCEL_BIAS_SPEED_MAX_MPS      (0.08f)   /* 低速才学偏置 */
#define ODOMETER_ACCEL_BIAS_GYRO_MAX_DPS       (6.0f)    /* 角速率阈值 */
#define ODOMETER_ACCEL_BIAS_TILT_RATE_MAX_DPS  (8.0f)    /* 倾斜速率阈值 */
#define ODOMETER_ACCEL_BIAS_NORM_MIN_G         (0.94f)   /* 加速度计模长下限 */
#define ODOMETER_ACCEL_BIAS_NORM_MAX_G         (1.06f)   /* 加速度计模长上限 */

/* ===== 颠簸/碰撞检测阈值 ===== */
#define ODOMETER_BUMP_HOLD_TICKS               (80U)     /* 颠簸后屏蔽时间，约 0.8s */
#define ODOMETER_BUMP_GYRO_DPS                 (25.0f)   /* 角速率触发阈值 */
#define ODOMETER_BUMP_TILT_RATE_DPS            (25.0f)   /* 倾斜速率触发阈值 */
#define ODOMETER_BUMP_NORM_MIN_G               (0.85f)   /* 撞击加速度下限 */
#define ODOMETER_BUMP_NORM_MAX_G               (1.15f)   /* 撞击加速度上限 */

/* ===== 粗糙路面检测（地面不平时放松编码器信任） ===== */
#define ODOMETER_ROUGH_HISTORY_SIZE            (11U)     /* 滑动窗口长度 */
#define ODOMETER_ROUGH_TILT_DEG                (3.8f)    /* 倾斜角触发阈值 */
#define ODOMETER_ROUGH_GYRO_DPS                (80.0f)   /* 角速率触发阈值 */
#define ODOMETER_ROUGH_WHEEL_COUNT             (20.0f)   /* 轮速高通触发阈值 */
#define ODOMETER_ROUGH_DUAL_AXIS_MPS           (0.05f)   /* 双轴最低速度要求 */
#define ODOMETER_ROUGH_DUAL_RATIO              (0.10f)   /* 双轴比例最低要求 */
#define ODOMETER_WHEEL_HIGHPASS_TAU_S          (0.08f)   /* 轮速高通滤波时间常数 */
#define ODOMETER_ROUGH_PRELOAD_TICKS           (20U)     /* 粗糙检测预加载延时 */
#define ODOMETER_ROUGH_RELAX_HOLD_TICKS        (70U)     /* 放松状态持续时间 */
#define ODOMETER_ROUGH_RELAX_EDGE_TICKS        (21U)     /* 放松权重渐变边缘 */
#define ODOMETER_ROUGH_RELAX_FORWARD_GAIN      (0.66f)   /* 前向放松增益（0~1） */
#define ODOMETER_ROUGH_RELAX_STRAFE_GAIN       (0.59f)   /* 横向放松增益（0~1） */

/* 二维向量，forward/strafe 方向 */
typedef struct
{
    float forward;
    float strafe;
} odometer_vec2_t;

/* 里程计内部滤波器状态（仅本文件使用） */
typedef struct
{
    odometer_vec2_t velocity_mps;               /* 融合后速度 */
    odometer_vec2_t prev_encoder_velocity_mps;   /* 上一帧编码器速度（用于算加速度和反转） */
    odometer_vec2_t accel_bias_mps2;             /* 加速度计偏置（低通学习值） */
    float yaw_zero_rad;                          /* 航向零点 */
    float prev_yaw_delta_rad;                    /* 上一帧航向偏移 */
    float roll_zero_rad;                         /* 横滚零点 */
    float pitch_zero_rad;                        /* 俯仰零点 */
    float prev_roll_rad;                         /* 上一帧横滚角 */
    float prev_pitch_rad;                        /* 上一帧俯仰角 */
    float alpha;                                 /* 编码器/加速度计融合系数 */
    float rough_tilt_history_deg[ODOMETER_ROUGH_HISTORY_SIZE];   /* 粗糙路面：倾斜角历史 */
    float rough_gyro_history_dps[ODOMETER_ROUGH_HISTORY_SIZE];   /* 粗糙路面：角速率历史 */
    float rough_wheel_history_count[ODOMETER_ROUGH_HISTORY_SIZE]; /* 粗糙路面：轮速高通历史 */
    float wheel_count_lpf[4];                    /* 4轮低通滤波器状态 */
    uint16_t bump_hold_ticks;                    /* 颠簸屏蔽倒计时 */
    uint16_t rough_relax_ticks;                  /* 粗糙路面放松倒计时 */
    uint16_t startup_hold_ticks;                 /* 启动稳定倒计时 */
    uint8_t rough_history_index;                 /* 环形缓冲区写指针 */
    uint8_t rough_history_count;                 /* 已填充数量 */
    uint8_t tilt_ready;                          /* 倾斜零点已初始化 */
    uint8_t yaw_ready;                           /* 航向零点已初始化 */
} odometer_filter_state_t;

odometer_data_t g_odometer = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

static odometer_filter_state_t g_odometer_filter;

/* 二维向量模长 */
static float odometer_vec_norm(odometer_vec2_t value)
{
    return sqrtf((value.forward * value.forward) + (value.strafe * value.strafe));
}

static float odometer_vec3_norm(float x, float y, float z)
{
    return sqrtf((x * x) + (y * y) + (z * z));
}

/* 角度归一化到 [-PI, PI) */
static float odometer_normalize_angle(float angle)
{
    while(angle > ODOMETER_PI)
    {
        angle -= ODOMETER_TWO_PI;
    }

    while(angle < -ODOMETER_PI)
    {
        angle += ODOMETER_TWO_PI;
    }

    return angle;
}

/* 读4轮编码器增量，解算成前后/横向脉冲数
 * 麦克纳姆轮解算：forward = (LF+RF+LR+RR)/4, strafe = (-LF+RF+LR-RR)/4 */
static odometer_vec2_t odometer_get_encoder_delta_count(void)
{
    odometer_vec2_t count;
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

    count.forward = (left_front + right_front + left_rear + right_rear) * 0.25f;
    count.strafe = (-left_front + right_front + left_rear - right_rear) * 0.25f;
    return count;
}

/* 轮速高通滤波，返回4轮高通绝对值的最大值
 * 用于粗糙路面检测：高通值大 = 轮子在跳 */
static float odometer_get_encoder_wheel_highpass_count(void)
{
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;
    float highpass_abs_max;
    float beta;

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();
    beta = ODOMETER_UPDATE_DT_S / (ODOMETER_WHEEL_HIGHPASS_TAU_S + ODOMETER_UPDATE_DT_S);

    g_odometer_filter.wheel_count_lpf[0] += beta * (left_front - g_odometer_filter.wheel_count_lpf[0]);
    g_odometer_filter.wheel_count_lpf[1] += beta * (right_front - g_odometer_filter.wheel_count_lpf[1]);
    g_odometer_filter.wheel_count_lpf[2] += beta * (left_rear - g_odometer_filter.wheel_count_lpf[2]);
    g_odometer_filter.wheel_count_lpf[3] += beta * (right_rear - g_odometer_filter.wheel_count_lpf[3]);

    highpass_abs_max = car_math_absf(left_front - g_odometer_filter.wheel_count_lpf[0]);
    highpass_abs_max = car_math_maxf(highpass_abs_max,
                                     car_math_absf(right_front - g_odometer_filter.wheel_count_lpf[1]));
    highpass_abs_max = car_math_maxf(highpass_abs_max,
                                     car_math_absf(left_rear - g_odometer_filter.wheel_count_lpf[2]));
    highpass_abs_max = car_math_maxf(highpass_abs_max,
                                     car_math_absf(right_rear - g_odometer_filter.wheel_count_lpf[3]));
    return highpass_abs_max;
}

/* 清空粗糙路面检测历史缓冲区 */
static void odometer_rough_history_clear(void)
{
    uint8_t i;

    for(i = 0U; i < ODOMETER_ROUGH_HISTORY_SIZE; i++)
    {
        g_odometer_filter.rough_tilt_history_deg[i] = 0.0f;
        g_odometer_filter.rough_gyro_history_dps[i] = 0.0f;
        g_odometer_filter.rough_wheel_history_count[i] = 0.0f;
    }

    for(i = 0U; i < 4U; i++)
    {
        g_odometer_filter.wheel_count_lpf[i] = 0.0f;
    }

    g_odometer_filter.rough_history_index = 0U;
    g_odometer_filter.rough_history_count = 0U;
}

static void odometer_rough_history_push(float tilt_deg,
                                        float gyro_dps,
                                        float wheel_count)
{
    g_odometer_filter.rough_tilt_history_deg[g_odometer_filter.rough_history_index] = tilt_deg;
    g_odometer_filter.rough_gyro_history_dps[g_odometer_filter.rough_history_index] = gyro_dps;
    g_odometer_filter.rough_wheel_history_count[g_odometer_filter.rough_history_index] = wheel_count;

    g_odometer_filter.rough_history_index++;
    if(g_odometer_filter.rough_history_index >= ODOMETER_ROUGH_HISTORY_SIZE)
    {
        g_odometer_filter.rough_history_index = 0U;
    }

    if(g_odometer_filter.rough_history_count < ODOMETER_ROUGH_HISTORY_SIZE)
    {
        g_odometer_filter.rough_history_count++;
    }
}

static float odometer_rough_history_max(const float *history)
{
    float value_max;
    uint8_t i;

    value_max = 0.0f;
    for(i = 0U; i < g_odometer_filter.rough_history_count; i++)
    {
        value_max = car_math_maxf(value_max, history[i]);
    }

    return value_max;
}

/* 编码器脉冲 -> 速度(m/s) */
static odometer_vec2_t odometer_get_encoder_velocity(odometer_vec2_t delta_count)
{
    odometer_vec2_t velocity;

    velocity.forward = delta_count.forward / ODOMETER_FORWARD_COUNT_PER_METER / ODOMETER_UPDATE_DT_S;
    velocity.strafe = delta_count.strafe / ODOMETER_STRAFE_COUNT_PER_METER_ABS / ODOMETER_UPDATE_DT_S;
    return velocity;
}

/* 学习加速度计偏置
 * 条件：低速 + 低角速率 + 加速度计模长在 0.94~1.06g 内 + 无颠簸
 * 满足条件时用 LPF 慢慢收敛偏置值 */
static void odometer_update_accel_bias(odometer_vec2_t encoder_velocity,
                                       odometer_vec2_t accel_mps2,
                                       float *gyro_norm_dps_out,
                                       float *tilt_rate_dps_out)
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float accel_norm_g;
    float gyro_norm_dps;
    float roll_rad;
    float pitch_rad;
    float roll_step_rad;
    float pitch_step_rad;
    float tilt_rate_dps;
    float speed;
    uint8_t bump_sample;

    speed = odometer_vec_norm(encoder_velocity);

    accel_x_g = 0.0f;
    accel_y_g = 0.0f;
    accel_z_g = 0.0f;
    gyro_x_dps = 0.0f;
    gyro_y_dps = 0.0f;
    gyro_z_dps = 0.0f;
    AccelCalibration_GetCorrectedSpecificForceG(&accel_x_g, &accel_y_g, &accel_z_g);
    AccelCalibration_GetBodyGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

    accel_norm_g = odometer_vec3_norm(accel_x_g, accel_y_g, accel_z_g);
    gyro_norm_dps = odometer_vec3_norm(gyro_x_dps, gyro_y_dps, gyro_z_dps);
    roll_rad = g_euler.roll * ODOMETER_DEG_TO_RAD;
    pitch_rad = g_euler.pitch * ODOMETER_DEG_TO_RAD;

    if(gyro_norm_dps_out != 0)
    {
        *gyro_norm_dps_out = gyro_norm_dps;
    }

    if(0U == g_odometer_filter.tilt_ready)
    {
        g_odometer_filter.prev_roll_rad = roll_rad;
        g_odometer_filter.prev_pitch_rad = pitch_rad;
        g_odometer_filter.tilt_ready = 1U;
        if(tilt_rate_dps_out != 0)
        {
            *tilt_rate_dps_out = 0.0f;
        }
        return;
    }

    roll_step_rad = odometer_normalize_angle(roll_rad - g_odometer_filter.prev_roll_rad);
    pitch_step_rad = odometer_normalize_angle(pitch_rad - g_odometer_filter.prev_pitch_rad);
    tilt_rate_dps = odometer_vec3_norm(roll_step_rad, pitch_step_rad, 0.0f) *
                    ODOMETER_RAD_TO_DEG / ODOMETER_UPDATE_DT_S;
    if(tilt_rate_dps_out != 0)
    {
        *tilt_rate_dps_out = tilt_rate_dps;
    }

    bump_sample = ((gyro_norm_dps > ODOMETER_BUMP_GYRO_DPS) ||
                   (tilt_rate_dps > ODOMETER_BUMP_TILT_RATE_DPS) ||
                   (accel_norm_g < ODOMETER_BUMP_NORM_MIN_G) ||
                   (accel_norm_g > ODOMETER_BUMP_NORM_MAX_G)) ? 1U : 0U;
    if(0U != bump_sample)
    {
        g_odometer_filter.bump_hold_ticks = ODOMETER_BUMP_HOLD_TICKS;
    }
    else if(g_odometer_filter.bump_hold_ticks > 0U)
    {
        g_odometer_filter.bump_hold_ticks--;
    }

    if((speed < ODOMETER_ACCEL_BIAS_SPEED_MAX_MPS) &&
       (gyro_norm_dps < ODOMETER_ACCEL_BIAS_GYRO_MAX_DPS) &&
       (tilt_rate_dps < ODOMETER_ACCEL_BIAS_TILT_RATE_MAX_DPS) &&
       (accel_norm_g >= ODOMETER_ACCEL_BIAS_NORM_MIN_G) &&
       (accel_norm_g <= ODOMETER_ACCEL_BIAS_NORM_MAX_G) &&
       (0U == g_odometer_filter.bump_hold_ticks))
    {
        g_odometer_filter.accel_bias_mps2.forward =
            car_filter_lpf1_apply(g_odometer_filter.accel_bias_mps2.forward,
                                accel_mps2.forward,
                                ODOMETER_UPDATE_DT_S,
                                1.2f);
        g_odometer_filter.accel_bias_mps2.strafe =
            car_filter_lpf1_apply(g_odometer_filter.accel_bias_mps2.strafe,
                                accel_mps2.strafe,
                                ODOMETER_UPDATE_DT_S,
                                1.2f);
    }

    g_odometer_filter.prev_roll_rad = roll_rad;
    g_odometer_filter.prev_pitch_rad = pitch_rad;
}

/* 粗糙路面状态更新
 * 检测条件：双轴运动 + 窗口内倾斜/角速率/轮速高通都超阈值
 * 触发后进入放松态，允许编码器更接近原始值 */
static void odometer_update_rough_relax_state(odometer_vec2_t raw_encoder_velocity,
                                              float gyro_norm_dps,
                                              float tilt_rate_dps)
{
    float tilt_deg;
    float roll_delta_rad;
    float pitch_delta_rad;
    float wheel_highpass_count;
    float tilt_window_max;
    float gyro_window_max;
    float wheel_window_max;
    float dual_axis_speed;
    float speed_norm;
    float dual_axis_ratio;

    roll_delta_rad = odometer_normalize_angle((g_euler.roll * ODOMETER_DEG_TO_RAD) -
                                              g_odometer_filter.roll_zero_rad);
    pitch_delta_rad = odometer_normalize_angle((g_euler.pitch * ODOMETER_DEG_TO_RAD) -
                                               g_odometer_filter.pitch_zero_rad);
    tilt_deg = odometer_vec3_norm(roll_delta_rad, pitch_delta_rad, 0.0f) * ODOMETER_RAD_TO_DEG;
    wheel_highpass_count = odometer_get_encoder_wheel_highpass_count();
    odometer_rough_history_push(tilt_deg, gyro_norm_dps, wheel_highpass_count);

    tilt_window_max = odometer_rough_history_max(g_odometer_filter.rough_tilt_history_deg);
    gyro_window_max = odometer_rough_history_max(g_odometer_filter.rough_gyro_history_dps);
    wheel_window_max = odometer_rough_history_max(g_odometer_filter.rough_wheel_history_count);
    dual_axis_speed = car_math_minf(car_math_absf(raw_encoder_velocity.forward),
                                   car_math_absf(raw_encoder_velocity.strafe));
    speed_norm = odometer_vec_norm(raw_encoder_velocity);
    dual_axis_ratio = dual_axis_speed / (speed_norm + ODOMETER_EPSILON);

    if((dual_axis_speed > ODOMETER_ROUGH_DUAL_AXIS_MPS) &&
       (dual_axis_ratio > ODOMETER_ROUGH_DUAL_RATIO) &&
       (tilt_window_max > ODOMETER_ROUGH_TILT_DEG) &&
       (gyro_window_max > ODOMETER_ROUGH_GYRO_DPS) &&
       (wheel_window_max > ODOMETER_ROUGH_WHEEL_COUNT))
    {
        g_odometer_filter.rough_relax_ticks =
            ODOMETER_ROUGH_PRELOAD_TICKS + ODOMETER_ROUGH_RELAX_HOLD_TICKS;
    }
    else if(g_odometer_filter.rough_relax_ticks > 0U)
    {
        g_odometer_filter.rough_relax_ticks--;
    }

    (void)tilt_rate_dps;
}

/* 粗糙路面放松权重，0~1，带渐入渐出 */
static float odometer_get_rough_relax_weight(void)
{
    float ticks;

    if(0U == g_odometer_filter.rough_relax_ticks)
    {
        return 0.0f;
    }

    if(g_odometer_filter.rough_relax_ticks >= ODOMETER_ROUGH_RELAX_EDGE_TICKS)
    {
        return 1.0f;
    }

    ticks = (float)g_odometer_filter.rough_relax_ticks;
    return car_math_clampf(ticks / (float)ODOMETER_ROUGH_RELAX_EDGE_TICKS, 0.0f, 1.0f);
}

void odometer_init(void)
{
    odometer_reset();
}

void odometer_reset(void)
{
    g_odometer.forward_distance = 0.0f;
    g_odometer.strafe_distance = 0.0f;
    g_odometer.travel_distance = 0.0f;
    g_odometer.forward_velocity_mps = 0.0f;
    g_odometer.strafe_velocity_mps = 0.0f;

    g_odometer_filter.velocity_mps.forward = 0.0f;
    g_odometer_filter.velocity_mps.strafe = 0.0f;
    g_odometer_filter.prev_encoder_velocity_mps.forward = 0.0f;
    g_odometer_filter.prev_encoder_velocity_mps.strafe = 0.0f;
    g_odometer_filter.accel_bias_mps2.forward = 0.0f;
    g_odometer_filter.accel_bias_mps2.strafe = 0.0f;
    g_odometer_filter.yaw_zero_rad = 0.0f;
    g_odometer_filter.prev_yaw_delta_rad = 0.0f;
    g_odometer_filter.roll_zero_rad = 0.0f;
    g_odometer_filter.pitch_zero_rad = 0.0f;
    g_odometer_filter.prev_roll_rad = 0.0f;
    g_odometer_filter.prev_pitch_rad = 0.0f;
    g_odometer_filter.alpha = 0.0f;
    g_odometer_filter.bump_hold_ticks = 0U;
    g_odometer_filter.rough_relax_ticks = 0U;
    g_odometer_filter.startup_hold_ticks = ODOMETER_STARTUP_HOLD_TICKS;
    odometer_rough_history_clear();
    g_odometer_filter.tilt_ready = 0U;
    g_odometer_filter.yaw_ready = 0U;
}

void odometer_update_100HZ(void)
{
    odometer_vec2_t delta_count;
    odometer_vec2_t encoder_velocity;
    odometer_vec2_t raw_encoder_velocity;
    odometer_vec2_t encoder_accel;
    odometer_vec2_t accel_raw;
    odometer_vec2_t accel_corrected;
    odometer_vec2_t distance_delta;
    float accel_z_mps2;
    float qvx;
    float qvy;
    float qrx;
    float qry;
    float reverse_forward;
    float reverse_strafe;
    float yaw_now_rad;
    float yaw_delta_rad;
    float yaw_step_rad;
    float yaw_rate_abs;
    float yaw_risk;
    float dual_axis;
    float scale_forward;
    float scale_strafe;
    float risk;
    float alpha_raw;
    float predicted_forward;
    float predicted_strafe;
    float fused_forward;
    float fused_strafe;
    float yaw_for_projection;
    float cos_yaw;
    float sin_yaw;
    float gyro_norm_dps;
    float tilt_rate_dps;
    float rough_relax_weight;

    /* --- 1. 读编码器算原始速度 --- */
    delta_count = odometer_get_encoder_delta_count();
    encoder_velocity = odometer_get_encoder_velocity(delta_count);
    raw_encoder_velocity = encoder_velocity;

    /* --- 2. 启动稳定期：清零所有状态，丢弃数据 --- */
    if(g_odometer_filter.startup_hold_ticks > 0U)
    {
        yaw_now_rad = g_euler.yaw * ODOMETER_DEG_TO_RAD;
        g_odometer.forward_distance = 0.0f;
        g_odometer.strafe_distance = 0.0f;
        g_odometer.travel_distance = 0.0f;
        g_odometer.forward_velocity_mps = 0.0f;
        g_odometer.strafe_velocity_mps = 0.0f;
        g_odometer_filter.velocity_mps.forward = 0.0f;
        g_odometer_filter.velocity_mps.strafe = 0.0f;
        g_odometer_filter.prev_encoder_velocity_mps = raw_encoder_velocity;
        g_odometer_filter.yaw_zero_rad = yaw_now_rad;
        g_odometer_filter.prev_yaw_delta_rad = 0.0f;
        g_odometer_filter.roll_zero_rad = g_euler.roll * ODOMETER_DEG_TO_RAD;
        g_odometer_filter.pitch_zero_rad = g_euler.pitch * ODOMETER_DEG_TO_RAD;
        g_odometer_filter.prev_roll_rad = g_euler.roll * ODOMETER_DEG_TO_RAD;
        g_odometer_filter.prev_pitch_rad = g_euler.pitch * ODOMETER_DEG_TO_RAD;
        g_odometer_filter.alpha = 0.0f;
        g_odometer_filter.bump_hold_ticks = 0U;
        g_odometer_filter.rough_relax_ticks = 0U;
        odometer_rough_history_clear();
        g_odometer_filter.tilt_ready = 1U;
        g_odometer_filter.yaw_ready = 1U;
        g_odometer_filter.startup_hold_ticks--;
        return;
    }

    /* --- 3. 读加速度计，学习偏置，算修正后加速度 --- */
    accel_raw.forward = 0.0f;
    accel_raw.strafe = 0.0f;
    accel_z_mps2 = 0.0f;
    AccelCalibration_GetBodyAccelMps2(&accel_raw.forward, &accel_raw.strafe, &accel_z_mps2);
    gyro_norm_dps = 0.0f;
    tilt_rate_dps = 0.0f;
    odometer_update_accel_bias(encoder_velocity, accel_raw, &gyro_norm_dps, &tilt_rate_dps);
    odometer_update_rough_relax_state(raw_encoder_velocity, gyro_norm_dps, tilt_rate_dps);
    accel_corrected.forward = accel_raw.forward - g_odometer_filter.accel_bias_mps2.forward;
    accel_corrected.strafe = accel_raw.strafe - g_odometer_filter.accel_bias_mps2.strafe;

    /* --- 4. 编码器微分加速度（用于残差计算） --- */
    encoder_accel.forward = (encoder_velocity.forward -
                             g_odometer_filter.prev_encoder_velocity_mps.forward) / ODOMETER_UPDATE_DT_S;
    encoder_accel.strafe = (encoder_velocity.strafe -
                            g_odometer_filter.prev_encoder_velocity_mps.strafe) / ODOMETER_UPDATE_DT_S;

    /* --- 5. 偏航角处理（相对零点的增量） --- */
    yaw_now_rad = g_euler.yaw * ODOMETER_DEG_TO_RAD;
    if(0U == g_odometer_filter.yaw_ready)
    {
        g_odometer_filter.yaw_zero_rad = yaw_now_rad;
        g_odometer_filter.prev_yaw_delta_rad = 0.0f;
        g_odometer_filter.yaw_ready = 1U;
    }
    yaw_delta_rad = odometer_normalize_angle(yaw_now_rad - g_odometer_filter.yaw_zero_rad);
    yaw_step_rad = odometer_normalize_angle(yaw_delta_rad - g_odometer_filter.prev_yaw_delta_rad);
    yaw_rate_abs = car_math_absf(yaw_step_rad) / ODOMETER_UPDATE_DT_S;

    /* --- 6. 各种风险因子归一化到 [0,2] --- */
    qvx = car_math_clampf(car_math_absf(encoder_velocity.forward) / ODOMETER_V_REF_MPS, 0.0f, 2.0f);
    qvy = car_math_clampf(car_math_absf(encoder_velocity.strafe) / ODOMETER_V_REF_MPS, 0.0f, 2.0f);
    qrx = car_math_clampf(car_math_absf(encoder_accel.forward - accel_corrected.forward) /
                          ODOMETER_RESIDUAL_REF_MPS2,
                          0.0f,
                          2.0f);
    qry = car_math_clampf(car_math_absf(encoder_accel.strafe - accel_corrected.strafe) /
                          ODOMETER_RESIDUAL_REF_MPS2,
                          0.0f,
                          2.0f);
    reverse_forward = car_math_clampf(-(encoder_velocity.forward *
                                        g_odometer_filter.prev_encoder_velocity_mps.forward) /
                                      (ODOMETER_REVERSE_REF_MPS * ODOMETER_REVERSE_REF_MPS),
                                      0.0f,
                                      2.0f);
    reverse_strafe = car_math_clampf(-(encoder_velocity.strafe *
                                       g_odometer_filter.prev_encoder_velocity_mps.strafe) /
                                     (ODOMETER_REVERSE_REF_MPS * ODOMETER_REVERSE_REF_MPS),
                                     0.0f,
                                     2.0f);
    yaw_risk = car_math_clampf(yaw_rate_abs / ODOMETER_YAW_RATE_REF_RPS, 0.0f, 2.0f);
    dual_axis = car_math_clampf(car_math_minf(car_math_absf(encoder_velocity.forward),
                                             car_math_absf(encoder_velocity.strafe)) / ODOMETER_V_REF_MPS,
                                0.0f,
                                1.5f);

    /* --- 7. 编码器速度衰减系数（风险越大衰减越多） --- */
    scale_forward = 1.0f /
                    (1.0f +
                     (ODOMETER_KX_SPEED * powf(qvx, ODOMETER_SPEED_POWER_X)) +
                     (ODOMETER_KX_RESIDUAL * powf(qrx, ODOMETER_RISK_GAMMA)) +
                     (ODOMETER_KX_REVERSE * reverse_forward) +
                     (ODOMETER_KX_YAW * yaw_risk) +
                     (ODOMETER_K_DUAL_AXIS * dual_axis) +
                     (ODOMETER_CROSS_AXIS * powf(qvy, ODOMETER_SPEED_POWER_Y) *
                      car_math_clampf(qrx, 0.0f, 1.0f)));
    scale_strafe = 1.0f /
                   (1.0f +
                    (ODOMETER_KY_SPEED * powf(qvy, ODOMETER_SPEED_POWER_Y)) +
                    (ODOMETER_KY_RESIDUAL * powf(qry, ODOMETER_RISK_GAMMA)) +
                    (ODOMETER_KY_REVERSE * reverse_strafe) +
                    (ODOMETER_KY_YAW * yaw_risk) +
                    (ODOMETER_K_DUAL_AXIS * dual_axis) +
                    (ODOMETER_CROSS_AXIS * powf(qvx, ODOMETER_SPEED_POWER_X) *
                     car_math_clampf(qry, 0.0f, 1.0f)));

    /* --- 8. 死区 + 粗糙路面放松 --- */
    encoder_velocity.forward = car_math_soft_deadband(encoder_velocity.forward * scale_forward,
                                                      ODOMETER_DEAD_FORWARD_MPS);
    encoder_velocity.strafe = car_math_soft_deadband(encoder_velocity.strafe * scale_strafe,
                                                     ODOMETER_DEAD_STRAFE_MPS);
    rough_relax_weight = odometer_get_rough_relax_weight();
    if(rough_relax_weight > 0.0f)
    {
        encoder_velocity.forward +=
            (raw_encoder_velocity.forward - encoder_velocity.forward) *
            ODOMETER_ROUGH_RELAX_FORWARD_GAIN * rough_relax_weight;
        encoder_velocity.strafe +=
            (raw_encoder_velocity.strafe - encoder_velocity.strafe) *
            ODOMETER_ROUGH_RELAX_STRAFE_GAIN * rough_relax_weight;
    }

    /* --- 9. 综合风险 -> 融合系数 alpha（越大越信加速度计） --- */
    risk = car_math_clampf((0.30f * car_math_maxf(qvx, qvy)) +
                           (0.45f * car_math_maxf(qrx, qry)) +
                           (0.15f * car_math_maxf(reverse_forward, reverse_strafe)) +
                           (0.10f * yaw_risk),
                           0.0f,
                           1.0f);
    alpha_raw = ODOMETER_ALPHA_MAX * risk;
    g_odometer_filter.alpha = car_filter_lpf1_apply(g_odometer_filter.alpha,
                                                  alpha_raw,
                                                  ODOMETER_UPDATE_DT_S,
                                                  ODOMETER_ALPHA_TAU_S);

    /* --- 10. 编码器 + 加速度计融合 --- */
    predicted_forward = g_odometer_filter.velocity_mps.forward +
                        (accel_corrected.forward * ODOMETER_UPDATE_DT_S);
    predicted_strafe = g_odometer_filter.velocity_mps.strafe +
                       (accel_corrected.strafe * ODOMETER_UPDATE_DT_S);
    fused_forward = (g_odometer_filter.alpha * predicted_forward) +
                    ((1.0f - g_odometer_filter.alpha) * encoder_velocity.forward);
    fused_strafe = (g_odometer_filter.alpha * predicted_strafe) +
                   ((1.0f - g_odometer_filter.alpha) * encoder_velocity.strafe);

    /* --- 11. 偏航投影 + 累积位移 --- */
    yaw_for_projection = yaw_delta_rad * ODOMETER_YAW_GAIN;
    cos_yaw = cosf(yaw_for_projection);
    sin_yaw = sinf(yaw_for_projection);
    distance_delta.forward = ((cos_yaw * fused_forward) - (sin_yaw * fused_strafe)) *
                             ODOMETER_UPDATE_DT_S;
    distance_delta.strafe = ((sin_yaw * fused_forward) + (cos_yaw * fused_strafe)) *
                            ODOMETER_UPDATE_DT_S;

    g_odometer.forward_distance += distance_delta.forward;
    g_odometer.strafe_distance += distance_delta.strafe;
    g_odometer.travel_distance += odometer_vec_norm(distance_delta);
    g_odometer.forward_velocity_mps = fused_forward;
    g_odometer.strafe_velocity_mps = fused_strafe;

    g_odometer_filter.velocity_mps.forward = fused_forward;
    g_odometer_filter.velocity_mps.strafe = fused_strafe;
    g_odometer_filter.prev_encoder_velocity_mps = raw_encoder_velocity;
    g_odometer_filter.prev_yaw_delta_rad = yaw_delta_rad;
}
