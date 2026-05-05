/*********************************************************************************************************************
* 四电机速度环控制模块 - 头文件
*
* 功能：使用四个增量式PID实例实现四个电机速度闭环控制，PID参数由菜单统一配置
********************************************************************************************************************/

#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "zf_common_headfile.h"
#include "pid.h"

extern IncrementPID wheel_left_front_pid;
extern IncrementPID wheel_right_front_pid;
extern IncrementPID wheel_left_rear_pid;
extern IncrementPID wheel_right_rear_pid;

/**
 * @brief 初始化四电机速度环
 */
void control_speed_loop_init(void);

/**
 * @brief 更新四电机速度环
 * @param left_front_target  左前轮目标速度，单位为编码器周期计数
 * @param right_front_target 右前轮目标速度，单位为编码器周期计数
 * @param left_rear_target   左后轮目标速度，单位为编码器周期计数
 * @param right_rear_target  右后轮目标速度，单位为编码器周期计数
 */
void control_speed_loop_update(float left_front_target, float right_front_target,
                               float left_rear_target, float right_rear_target);

/**
 * @brief 停止四电机速度环并清零输出
 */
void control_speed_loop_stop(void);

/**
 * @brief 复位四电机速度环PID状态
 */
void control_speed_loop_reset(void);

#endif
