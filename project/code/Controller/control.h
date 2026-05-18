/* 串级PID控制模块
 *
 * 架构（仿 fc_loop.c 风格）：
 *   25Hz → 航向角度环 + 航向保持
 *   50Hz → 航向角速度环
 *   100Hz → 四轮速度环 + 麦克纳姆解算 + 电机输出
 */

#include "zf_common_headfile.h"
#ifndef __CONTROL_H__
#define __CONTROL_H__

/* 四轮位置式PID */
extern PositionalPID wheel_left_front_pid;
extern PositionalPID wheel_right_front_pid;
extern PositionalPID wheel_left_rear_pid;
extern PositionalPID wheel_right_rear_pid;

/* 航向PID */
extern PositionalPID yaw_angle_pid;
extern PositionalPID yaw_rate_pid;

/* 调试中间变量 */
extern float control_yaw_angle_current;
extern float control_yaw_angle_output;
extern float control_yaw_rate_target;
extern float control_yaw_rate_current;
extern float control_yaw_rate_raw;
extern float control_yaw_rate_output;

void  Control_Init(void);
void  Control_Reset(void);
void  Control_Stop(void);
void  Control_YawHoldReset(void);
void  Control_25Hz(float rotate_target);
void  Control_50Hz(float rotate_target);
void  Control_100Hz(float forward, float strafe);
float Control_GetYawAngle(void);

#endif
