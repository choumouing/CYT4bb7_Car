/*********************************************************************************************************************
* CYT4BB 编码器控制模块 - 头文件
*
* 文件功能：封装正交编码器的初始化、读取功能
* 模块说明：
*   1. 支持最多 4 个正交编码器
*   2. 提供周期计数读取功能（基于定时中断）
*   3. 速度使用周期原始计数，里程使用累计计数
********************************************************************************************************************/

#include "zf_common_headfile.h"
#ifndef _ENCODER_CONTROL_H_
#define _ENCODER_CONTROL_H_



// ========================== 配置参数 ==========================

// 一阶卡尔曼滤波参数
#define ENCODER_KALMAN_PROCESS_NOISE       (0.25f)
#define ENCODER_KALMAN_MEASURE_NOISE       (4.0f)
#define ENCODER_KALMAN_ERROR_INIT          (1.0f)



// 引脚定义
// 编码器 M1 硬件引脚 (实际对应左前位置)
#define ENCODER_M1_INDEX    (TC_CH20_ENCODER)
#define ENCODER_M1_A        (TC_CH20_ENCODER_CH1_P08_1)
#define ENCODER_M1_B        (TC_CH20_ENCODER_CH2_P08_2)

// 编码器 M2 硬件引脚 (实际对应右前位置)
#define ENCODER_M2_INDEX    (TC_CH07_ENCODER)
#define ENCODER_M2_A        (TC_CH07_ENCODER_CH1_P07_6)
#define ENCODER_M2_B        (TC_CH07_ENCODER_CH2_P07_7)

// 编码器 M3 硬件引脚 (实际对应左后位置)
#define ENCODER_M3_INDEX    (TC_CH58_ENCODER)
#define ENCODER_M3_A        (TC_CH58_ENCODER_CH1_P17_3)
#define ENCODER_M3_B        (TC_CH58_ENCODER_CH2_P17_4)

// 编码器 M4 硬件引脚 (实际对应右后位置)
#define ENCODER_M4_INDEX    (TC_CH27_ENCODER)
#define ENCODER_M4_A        (TC_CH27_ENCODER_CH1_P19_2)
#define ENCODER_M4_B        (TC_CH27_ENCODER_CH2_P19_3)

// ========================== 数据结构 ==========================

typedef struct
{
    encoder_index_enum index;                   // 编码器索引
    encoder_channel1_enum ch1_pin;              // 通道 1 引脚
    encoder_channel2_enum ch2_pin;              // 通道 2 引脚

    int16_t count_raw;                          // 当前周期原始计数（速度）
    float count_filtered;                       // 当前周期滤波计数（速度）
    int32_t count_total;                        // 累计计数（里程）
    float kalman_p;                             // 卡尔曼估计误差协方差

    int8_t invert;
} encoder_data_t;

// ========================== 全局变量声明 ==========================


extern encoder_data_t encoder_left_front;             // 左编码器数据
extern encoder_data_t encoder_right_front;            // 右编码器数据
extern encoder_data_t encoder_left_rear;             // 左编码器数据
extern encoder_data_t encoder_right_rear;            // 右编码器数据

// ========================== 函数声明 ==========================

/**
 * @brief  初始化编码器模块
 * @return 无
 * @note   根据宏定义初始化左右编码器
 */
void encoder_control_init(void);

/**
 * @brief  读取编码器计数并更新数据（在定时中断中调用）
 * @return 无
 * @note
 *   1. 读取编码器计数值并清零
 *   2. 更新累计计数
 *   3. 应在 5ms 定时中断中调用
 */
void encoder_update_100HZ(void);

/**
 * @brief  获取左前编码器周期计数（速度）
 * @return 左前编码器周期计数
 */
int16_t encoder_get_left_front_count(void);

/**
 * @brief  获取右前编码器周期计数（速度）
 * @return 右前编码器周期计数
 */
int16_t encoder_get_right_front_count(void);

/**
 * @brief  获取左后编码器周期计数（速度）
 * @return 左后编码器周期计数
 */
int16_t encoder_get_left_rear_count(void);

/**
 * @brief  获取右后编码器周期计数（速度）
 * @return 右后编码器周期计数
 */
int16_t encoder_get_right_rear_count(void);

/**
 * @brief  获取左前编码器滤波后周期计数（速度）
 * @return 左前编码器滤波后周期计数
 */
float encoder_get_left_front_filtered_count(void);

/**
 * @brief  获取右前编码器滤波后周期计数（速度）
 * @return 右前编码器滤波后周期计数
 */
float encoder_get_right_front_filtered_count(void);

/**
 * @brief  获取左后编码器滤波后周期计数（速度）
 * @return 左后编码器滤波后周期计数
 */
float encoder_get_left_rear_filtered_count(void);

/**
 * @brief  获取右后编码器滤波后周期计数（速度）
 * @return 右后编码器滤波后周期计数
 */
float encoder_get_right_rear_filtered_count(void);

/**
 * @brief  获取左前编码器累计计数
 * @return 左前编码器累计计数
 */
int32_t encoder_get_left_front_total(void);

/**
 * @brief  获取右前编码器累计计数
 * @return 右前编码器累计计数
 */
int32_t encoder_get_right_front_total(void);

/**
 * @brief  获取左后编码器累计计数
 * @return 左前编码器累计计数
 */
int32_t encoder_get_left_rear_total(void);

/**
 * @brief  获取右后编码器累计计数
 * @return 右前编码器累计计数
 */
int32_t encoder_get_right_rear_total(void);

/**
 * @brief  清零左前编码器累计计数
 * @return 无
 */
void encoder_clear_left_front_total(void);

/**
 * @brief  清零右前编码器累计计数
 * @return 无
 */
void encoder_clear_right_front_total(void);

/**
 * @brief  清零左后编码器累计计数
 * @return 无
 */
void encoder_clear_left_rear_total(void);

/**
 * @brief  清零右后编码器累计计数
 * @return 无
 */
void encoder_clear_right_rear_total(void);

#endif // _ENCODER_CONTROL_H_
