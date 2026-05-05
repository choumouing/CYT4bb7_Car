/*********************************************************************************************************************
* CYT4BB 编码器控制模块 - 实现文件
*
* 文件功能：封装正交编码器的初始化、读取功能
********************************************************************************************************************/

#include "encoder_control.h"
#include <stdio.h>

// ========================== 全局变量定义 ==========================


// Physical wheel mapping after chassis wiring update:
// left front -> M3, right front -> M4, left rear -> M2, right rear -> M1.
encoder_data_t encoder_left_front = {
    .index = ENCODER_M3_INDEX,
    .ch1_pin = ENCODER_M3_A,
    .ch2_pin = ENCODER_M3_B,
    .count_raw = 0,
    .count_total = 0,
    .invert = 1                     //是否反转编码器正负
};



encoder_data_t encoder_right_front = {
    .index = ENCODER_M4_INDEX,
    .ch1_pin = ENCODER_M4_A,
    .ch2_pin = ENCODER_M4_B,
    .count_raw = 0,
    .count_total = 0,
    .invert = 0
};

encoder_data_t encoder_left_rear = {
    .index = ENCODER_M2_INDEX,
    .ch1_pin = ENCODER_M2_A,
    .ch2_pin = ENCODER_M2_B,
    .count_raw = 0,
    .count_total = 0,
    .invert = 1
};

encoder_data_t encoder_right_rear = {
    .index = ENCODER_M1_INDEX,
    .ch1_pin = ENCODER_M1_A,
    .ch2_pin = ENCODER_M1_B,
    .count_raw = 0,
    .count_total = 0,
    .invert = 0
};



/**
 * @brief  更新单个编码器的数据
 * @param  encoder:    编码器数据结构指针
 * @return 无
 */
static void encoder_update_single(encoder_data_t *encoder)
{
    // 读取编码器计数并清零
    int16_t count = encoder_get_count(encoder->index);
    encoder_clear_count(encoder->index);
    // 处理反向
    if (encoder->invert)count = -count;
    encoder->count_raw = count;
    // 更新累计计数
    encoder->count_total += encoder->count_raw;
}


/**
 * @brief  初始化编码器模块
 */
void encoder_control_init(void)
{
    encoder_quad_init(encoder_left_front.index, encoder_left_front.ch1_pin, encoder_left_front.ch2_pin);
    encoder_quad_init(encoder_right_front.index, encoder_right_front.ch1_pin, encoder_right_front.ch2_pin);
    encoder_quad_init(encoder_left_rear.index, encoder_left_rear.ch1_pin, encoder_left_rear.ch2_pin);
    encoder_quad_init(encoder_right_rear.index, encoder_right_rear.ch1_pin, encoder_right_rear.ch2_pin);
}
/**
 * @brief  读取编码器计数并更新数据
 */
void encoder_update(void)
{
    encoder_update_single(&encoder_left_front); 
    encoder_update_single(&encoder_right_front);
    encoder_update_single(&encoder_left_rear);
    encoder_update_single(&encoder_right_rear);
}
/**
 * @brief  获取左前编码器周期计数（速度）
 */
int16_t encoder_get_left_front_count(void)
{
    return encoder_left_front.count_raw;
}
/**
 * @brief  获取右前编码器周期计数（速度）
 */
int16_t encoder_get_right_front_count(void)
{
    return encoder_right_front.count_raw;
}
/**
 * @brief  获取左后编码器周期计数（速度）
 */
int16_t encoder_get_left_rear_count(void)
{
    return encoder_left_rear.count_raw;
}
/**
 * @brief  获取右后编码器周期计数（速度）
 */
int16_t encoder_get_right_rear_count(void)
{
    return encoder_right_rear.count_raw;
}
/**
 * @brief  获取左前编码器累计计数
 */
int32_t encoder_get_left_front_total(void)
{
    return encoder_left_front.count_total;
}
/**
 * @brief  获取右前编码器累计计数
 */
int32_t encoder_get_right_front_total(void)
{
    return encoder_right_front.count_total;
}
/**
 * @brief  获取左后编码器累计计数
 */
int32_t encoder_get_left_rear_total(void)
{
    return encoder_left_rear.count_total;
}
/**
 * @brief  获取右后编码器累计计数
 */
int32_t encoder_get_right_rear_total(void)
{
    return encoder_right_rear.count_total;
}
/**
 * @brief  清零左前编码器累计计数
 */
void encoder_clear_left_front_total(void)
{
    encoder_left_front.count_total = 0;
}
/**
 * @brief  清零右前编码器累计计数
 */
void encoder_clear_right_front_total(void)
{
    encoder_right_front.count_total = 0;
}  
/**
 * @brief  清零左后编码器累计计数
 */
void encoder_clear_left_rear_total(void)
{
    encoder_left_rear.count_total = 0;
}
/**
 * @brief  清零右后编码器累计计数
 */
void encoder_clear_right_rear_total(void)
{
    encoder_right_rear.count_total = 0;
}
