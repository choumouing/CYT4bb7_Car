/********************************************************************
 * 文件名  : IMU_Filtter.c
 * 模块    : IMU 输入链路滤波实现
 * 滤波链  :
 *   陀螺仪: AntiAlias(250Hz LPF) -> Notch0(161Hz) -> Notch1(320Hz) -> LPF(60Hz)
 *   加速度: Notch0(161Hz) -> Notch1(320Hz) -> LPF(12Hz)
 * 所有滤波器都是二阶 IIR（Butterworth 低通 / 陷波），Transposed Direct Form II
 ********************************************************************/

#include "IMU_Filtter.h"


#define IMU_FILTER_PI  (3.14159265359f)

/* ======================== 全局滤波输出 ======================== */
imudata_t g_imufilter_1000hz; /* 1kHz 输出，姿态/里程计/EKF 使用 */
imudata_t g_imudata_500hz;    /* 500Hz 预留（当前与 1kHz 共用） */
imudata_t g_imudata_250hz;    /* 250Hz 预留（当前与 1kHz 共用） */

/* ======================== 内部滤波器状态 ======================== */
static struct
{
    /* 陀螺仪链路：AntiAlias -> Notch0 -> Notch1 -> LPF */
    IMUBiquad_t gyro_anti_alias_lpf[IMU_AXIS_NUM]; /* 抗混叠低通，防高频折叠 */
    IMUBiquad_t gyro_notch0[IMU_AXIS_NUM];         /* 第一级陷波，滤 161Hz 电机振动 */
    IMUBiquad_t gyro_notch1[IMU_AXIS_NUM];         /* 第二级陷波，滤 320Hz 谐波 */
    IMUBiquad_t gyro_lpf[IMU_AXIS_NUM];            /* 主低通 60Hz，平滑输出 */

    /* 加速度计链路：Notch0 -> Notch1 -> LPF（不加抗混叠，频率更低） */
    IMUBiquad_t accel_notch0[IMU_AXIS_NUM];        /* 第一级陷波 */
    IMUBiquad_t accel_notch1[IMU_AXIS_NUM];        /* 第二级陷波 */
    IMUBiquad_t accel_lpf[IMU_AXIS_NUM];           /* 主低通 12Hz，保留低频加速度 */

    uint8_t initialized; /* 0=首帧未初始化（直通输入），1=正常滤波 */
} s_filt;

/* 执行一次二阶 IIR 滤波（Transposed Direct Form II）
 * f: 滤波器状态（系数 + 延迟），in: 当前输入
 * 返回: 滤波后输出值 */
static float IMUBiquad_Apply(IMUBiquad_t *f, float in)
{
    float out = f->b0 * in + f->d1;
    f->d1 = f->b1 * in - f->a1 * out + f->d2;
    f->d2 = f->b2 * in - f->a2 * out;
    return out;
}

/* 初始化二阶 Butterworth 低通滤波器系数
 * f: 目标状态，fs: 采样率 Hz，fc: 截止频率 Hz
 * Q 固定为 0.707（Butterworth 特征） */
static void IMUBiquad_InitLPF(IMUBiquad_t *f, float fs, float fc)
{
    float w0;
    float sw0;
    float cw0;
    float alpha;
    float a0;

    w0 = 2.0f * IMU_FILTER_PI * fc / fs;
    sw0 = sinf(w0);
    cw0 = cosf(w0);
    alpha = sw0 / (2.0f * 0.70710678f);
    a0 = 1.0f + alpha;

    f->b0 = (1.0f - cw0) * 0.5f / a0;
    f->b1 = (1.0f - cw0) / a0;
    f->b2 = (1.0f - cw0) * 0.5f / a0;
    f->a1 = (-2.0f * cw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

/* 初始化二阶陷波滤波器系数
 * f: 目标状态，fs: 采样率 Hz，fc: 中心频率 Hz，q: 品质因数（Q=f/BW） */
static void IMUBiquad_InitNotch(IMUBiquad_t *f, float fs, float fc, float q)
{
    float w0;
    float sw0;
    float cw0;
    float alpha;
    float a0;

    w0 = 2.0f * IMU_FILTER_PI * fc / fs;
    sw0 = sinf(w0);
    cw0 = cosf(w0);
    alpha = sw0 / (2.0f * q);
    a0 = 1.0f + alpha;

    f->b0 = 1.0f / a0;
    f->b1 = (-2.0f * cw0) / a0;
    f->b2 = 1.0f / a0;
    f->a1 = (-2.0f * cw0) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

/* 初始化 IMU 输入链路的全部滤波器
 * 调用方: IMU_Init_All()，上电时调用一次 */
void IMUFilter_Init(void)
{
    uint8_t axis;

    for (axis = 0U; axis < IMU_AXIS_NUM; axis++)
    {
        /* 陀螺仪链路：AntiAliasLPF -> Notch0 -> Notch1 -> LPF */
        IMUBiquad_InitLPF(&s_filt.gyro_anti_alias_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_ANTI_ALIAS_LPF_HZ);
        IMUBiquad_InitNotch(&s_filt.gyro_notch0[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH0_HZ, IMU_NOTCH0_Q);
        IMUBiquad_InitNotch(&s_filt.gyro_notch1[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH1_HZ, IMU_NOTCH1_Q);
        IMUBiquad_InitLPF(&s_filt.gyro_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_GYRO_LPF_HZ);

        /* 加速度计链路：Notch0 -> Notch1 -> LPF */
        IMUBiquad_InitNotch(&s_filt.accel_notch0[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH0_HZ, IMU_NOTCH0_Q);
        IMUBiquad_InitNotch(&s_filt.accel_notch1[axis], IMU_SAMPLE_RATE_HZ, IMU_NOTCH1_HZ, IMU_NOTCH1_Q);
        IMUBiquad_InitLPF(&s_filt.accel_lpf[axis], IMU_SAMPLE_RATE_HZ, IMU_ACCEL_LPF_HZ);
    }

    s_filt.initialized = 0U;
    g_imufilter_1000hz = (imudata_t){0};
    g_imudata_500hz = (imudata_t){0};
    g_imudata_250hz = (imudata_t){0};
}

/* 1kHz 主滤波更新
 * 输入原始陀螺仪 dps + 加速度计 g，经过：
 *   陀螺: AntiAlias(250Hz) -> Notch(161Hz) -> Notch(320Hz) -> LPF(60Hz)
 *   加速度: Notch(161Hz) -> Notch(320Hz) -> LPF(12Hz)
 * 输出到 g_imufilter_1000hz（供姿态解算和里程计使用） */
void IMUFilter_Update(float gx, float gy, float gz,
                      float ax, float ay, float az)
{
    float gyro_in[IMU_AXIS_NUM];
    float accel_in[IMU_AXIS_NUM];
    float gyro_out[IMU_AXIS_NUM];
    float accel_out[IMU_AXIS_NUM];
    uint8_t axis;

    gyro_in[0] = gx;
    gyro_in[1] = gy;
    gyro_in[2] = gz;
    accel_in[0] = ax;
    accel_in[1] = ay;
    accel_in[2] = az;

    if (0U == s_filt.initialized)
    {
        g_imufilter_1000hz = (imudata_t){gx, gy, gz, ax, ay, az};
        g_imudata_500hz = (imudata_t){gx, gy, gz, ax, ay, az};
        g_imudata_250hz = (imudata_t){gx, gy, gz, ax, ay, az};
        s_filt.initialized = 1U;
        return;
    }

    for (axis = 0U; axis < IMU_AXIS_NUM; axis++)
    {
        float gyro_stage_aa;
        float gyro_stage0;
        float gyro_stage1;
        float accel_stage0;
        float accel_stage1;

        /* 陀螺仪链路: 抗混叠(250Hz) -> 陷波1(161Hz) -> 陷波2(320Hz) -> 低通(60Hz) */
        gyro_stage_aa = IMUBiquad_Apply(&s_filt.gyro_anti_alias_lpf[axis], gyro_in[axis]);
        gyro_stage0 = IMUBiquad_Apply(&s_filt.gyro_notch0[axis], gyro_stage_aa);
        gyro_stage1 = IMUBiquad_Apply(&s_filt.gyro_notch1[axis], gyro_stage0);
        gyro_out[axis] = IMUBiquad_Apply(&s_filt.gyro_lpf[axis], gyro_stage1);

        /* 加速度计链路: 陷波1(161Hz) -> 陷波2(320Hz) -> 低通(12Hz) */
        accel_stage0 = IMUBiquad_Apply(&s_filt.accel_notch0[axis], accel_in[axis]);
        accel_stage1 = IMUBiquad_Apply(&s_filt.accel_notch1[axis], accel_stage0);
        accel_out[axis] = IMUBiquad_Apply(&s_filt.accel_lpf[axis], accel_stage1);
    }

    g_imufilter_1000hz.gyrox = gyro_out[0];
    g_imufilter_1000hz.gyroy = gyro_out[1];
    g_imufilter_1000hz.gyroz = gyro_out[2];
    g_imufilter_1000hz.accx = accel_out[0];
    g_imufilter_1000hz.accy = accel_out[1];
    g_imufilter_1000hz.accz = accel_out[2];

    g_imudata_500hz = g_imufilter_1000hz;
    g_imudata_250hz = g_imufilter_1000hz;
}
