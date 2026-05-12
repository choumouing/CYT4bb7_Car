/* 主循环调度模块 - 头文件
 *
 * 定时器标志：由PIT中断置1，主循环轮询清零并执行对应频率任务
 * 全局控制目标：各模式写入，100HZ速度环读取
 *   forward/strafe：编码器周期计数
 *   rotate：rad/s（角速度环输入）或编码器计数（直接旋转输入）
 */

#ifndef CAR_LOOP_H
#define CAR_LOOP_H

#include "zf_common_headfile.h"


extern volatile uint8_t timer_100HZ_flag;   // 100HZ定时器标志（PIT中断置位）
extern volatile uint8_t timer_50HZ_flag;    // 50HZ定时器标志
extern volatile uint8_t timer_25HZ_flag;    // 25HZ定时器标志
extern volatile uint16 g_tick_1000HZ;       // 1000HZ滴答计数（IMU更新用）

/* 全局控制目标（由car_mode_update_25HZ写入，speed_loop_100HZ读取） */
extern float car_forward_target;            // 前后速度目标（编码器计数，正=前）
extern float car_strafe_target;             // 左右速度目标（编码器计数，正=右）
extern float car_rotate_target;             // 旋转输入（rad/s，来自遥控器或保持为0）
extern uint8 car_control_enabled;           // 控制使能标志（遥控器决定）
extern uint8 car_emergency_stop_active;     // 紧急停车标志（1=紧急停）

/* 初始化所有外设和模块，启动1000HZ定时器 */
void car_loop_init(void);

/* 主循环轮询入口（while(1)中调用）
 * 按优先级检查各频率标志并执行对应任务
 * 注意：1000HZ任务有防堆积保护（最多追赶100次）
 */
void car_loop_poll(void);

#endif /* CAR_LOOP_H */
