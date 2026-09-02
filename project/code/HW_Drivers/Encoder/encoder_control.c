/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
/*********************************************************************************************************************
* CYT4BB 编码器控制模块 - 实现文件
*
* 文件功能：封装四路正交编码器的初始化、周期读取、一阶低通滤波
* 轮子到编码器映射：左前->M3, 右前->M4, 左后->M2, 右后->M1
********************************************************************************************************************/

#include "encoder_control.h"

// ========================== 全局变量定义 ==========================

/* 轮子到编码器通道映射（根据底盘接线确定）：左前->M3, 右前->M4, 左后->M2, 右后->M1 */
encoder_data_t encoder_left_front = {
    .index = ENCODER_M3_INDEX,
    .ch1_pin = ENCODER_M3_A,
    .ch2_pin = ENCODER_M3_B,
    .count_raw = 0,
    .count_filtered = 0.0f,
    .count_total = 0,
    .invert = 1                     //是否反转编码器正负
};



encoder_data_t encoder_right_front = {
    .index = ENCODER_M4_INDEX,
    .ch1_pin = ENCODER_M4_A,
    .ch2_pin = ENCODER_M4_B,
    .count_raw = 0,
    .count_filtered = 0.0f,
    .count_total = 0,
    .invert = 0
};

encoder_data_t encoder_left_rear = {
    .index = ENCODER_M2_INDEX,
    .ch1_pin = ENCODER_M2_A,
    .ch2_pin = ENCODER_M2_B,
    .count_raw = 0,
    .count_filtered = 0.0f,
    .count_total = 0,
    .invert = 0
};

encoder_data_t encoder_right_rear = {
    .index = ENCODER_M1_INDEX,
    .ch1_pin = ENCODER_M1_A,
    .ch2_pin = ENCODER_M1_B,
    .count_raw = 0,
    .count_filtered = 0.0f,
    .count_total = 0,
    .invert = 0
};


/*
 * 一阶低通滤波
 * 输入：原始脉冲计数（float）
 * 输出：平滑后的估计值，同时更新 encoder->count_filtered
 */
static float encoder_lowpass_filter(encoder_data_t *encoder, float measurement)
{
    encoder->count_filtered += ENCODER_LPF_ALPHA * (measurement - encoder->count_filtered);

    return encoder->count_filtered;
}


/*
 * 更新单个编码器：读取硬件计数 -> 清零 -> invert 取反 -> 一阶低通滤波 -> 累加里程
 * 每个编码器在 100Hz 中断中被调用一次
 */
static void encoder_update_single(encoder_data_t *encoder)
{
    int16_t count = encoder_get_count(encoder->index);
    encoder_clear_count(encoder->index);

    if (encoder->invert)
        count = -count;             // 接线反了？invert=1 自动取反

    encoder->count_raw = count;
    encoder_lowpass_filter(encoder, (float)encoder->count_raw);
    encoder->count_total += encoder->count_raw;     // 里程累计
}


/* 初始化四路 TC 正交编码器硬件 */
void encoder_control_init(void)
{
    encoder_quad_init(encoder_left_front.index, encoder_left_front.ch1_pin, encoder_left_front.ch2_pin);
    encoder_quad_init(encoder_right_front.index, encoder_right_front.ch1_pin, encoder_right_front.ch2_pin);
    encoder_quad_init(encoder_left_rear.index, encoder_left_rear.ch1_pin, encoder_left_rear.ch2_pin);
    encoder_quad_init(encoder_right_rear.index, encoder_right_rear.ch1_pin, encoder_right_rear.ch2_pin);
}
/* 100Hz 中断入口：更新四路编码器速度、滤波和里程 */
void encoder_update_100HZ(void)
{
    encoder_update_single(&encoder_left_front);
    encoder_update_single(&encoder_right_front);
    encoder_update_single(&encoder_left_rear);
    encoder_update_single(&encoder_right_rear);
}
/* 各轮原始速度读取（脉冲/周期） */
int16_t encoder_get_left_front_count(void)
{
    return encoder_left_front.count_raw;
}
int16_t encoder_get_right_front_count(void)
{
    return encoder_right_front.count_raw;
}
int16_t encoder_get_left_rear_count(void)
{
    return encoder_left_rear.count_raw;
}
int16_t encoder_get_right_rear_count(void)
{
    return encoder_right_rear.count_raw;
}

/* 各轮一阶低通滤波后速度 */
float encoder_get_left_front_filtered_count(void)
{
    return encoder_left_front.count_filtered;
}
float encoder_get_right_front_filtered_count(void)
{
    return encoder_right_front.count_filtered;
}
float encoder_get_left_rear_filtered_count(void)
{
    return encoder_left_rear.count_filtered;
}
float encoder_get_right_rear_filtered_count(void)
{
    return encoder_right_rear.count_filtered;
}

/* 各轮累计里程（脉冲数） */
int32_t encoder_get_left_front_total(void)
{
    return encoder_left_front.count_total;
}
int32_t encoder_get_right_front_total(void)
{
    return encoder_right_front.count_total;
}
int32_t encoder_get_left_rear_total(void)
{
    return encoder_left_rear.count_total;
}
int32_t encoder_get_right_rear_total(void)
{
    return encoder_right_rear.count_total;
}

/* 清零累计里程 */
void encoder_clear_left_front_total(void)
{
    encoder_left_front.count_total = 0;
}
void encoder_clear_right_front_total(void)
{
    encoder_right_front.count_total = 0;
}
void encoder_clear_left_rear_total(void)
{
    encoder_left_rear.count_total = 0;
}
void encoder_clear_right_rear_total(void)
{
    encoder_right_rear.count_total = 0;
}
