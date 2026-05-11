/*********************************************************************************************************************
* 四电机速度环控制模块 - 头文件
*
* 功能：使用四个增量式PID实例实现四个电机速度闭环控制，PID参数由菜单统一配置
********************************************************************************************************************/

#include "zf_common_headfile.h"
#ifndef __CONTROL_H__
#define __CONTROL_H__



extern IncrementPID wheel_left_front_pid;
extern IncrementPID wheel_right_front_pid;
extern IncrementPID wheel_left_rear_pid;
extern IncrementPID wheel_right_rear_pid;
extern PositionalPID yaw_angle_pid;
extern PositionalPID yaw_rate_pid;

extern float control_yaw_angle_current;
extern float control_yaw_angle_output;
extern float control_yaw_rate_target;
extern float control_yaw_rate_current;
extern float control_yaw_rate_raw;
extern float control_yaw_rate_output;

float control_get_current_yaw_angle(void);
void control_yaw_hold_reset(void);

/**
 * @brief 初始化四电机速度环
 */
void control_speed_loop_init(void);

/**
 * @brief 初始化Yaw角速度环和四轮速度环
 */
void control_cascade_init(void);

/**
 * @brief 更新Yaw角度环
 * @param yaw_angle_target 目标Yaw角度，单位 rad
 * @return 目标Yaw角速度，单位 rad/s
 */
float control_yaw_angle_loop_update_25HZ(float yaw_angle_target);

/**
 * @brief 更新Yaw角速度环
 * @param yaw_rate_target 目标Yaw角速度，单位 rad/s
 * @return 旋转分量目标，单位为编码器周期计数
 */
float control_yaw_rate_loop_update_50HZ(float yaw_rate_target);
void control_yaw_hold_update_25HZ(float rotate_target);

/**
 * @brief 使用当前Yaw角速度环输出更新四轮速度环
 * @param forward_target 前后方向目标速度，单位为编码器周期计数
 * @param strafe_target  左右方向目标速度，单位为编码器周期计数
 */
void control_cascade_speed_loop_update_100HZ(float forward_target, float strafe_target);

void control_cascade_speed_loop_update_with_rotate_100HZ(float forward_target, float strafe_target, float rotate_target);

/**
 * @brief 麦克纳姆轮串级控制
 * @param forward_target 前后方向目标速度，单位为编码器周期计数
 * @param strafe_target  左右方向目标速度，单位为编码器周期计数
 * @param yaw_rate_target 目标Yaw角速度，单位 rad/s
 */
void control_cascade_update_50HZ(float forward_target, float strafe_target, float yaw_rate_target);

/**
 * @brief 更新四电机速度环
 * @param left_front_target  左前轮目标速度，单位为编码器周期计数
 * @param right_front_target 右前轮目标速度，单位为编码器周期计数
 * @param left_rear_target   左后轮目标速度，单位为编码器周期计数
 * @param right_rear_target  右后轮目标速度，单位为编码器周期计数
 */
void control_speed_loop_update_100HZ(float left_front_target, float right_front_target,
                                     float left_rear_target, float right_rear_target);

/**
 * @brief 停止四电机速度环并清零输出
 */
void control_speed_loop_stop(void);

/**
 * @brief 停止串级控制并清零PID状态
 */
void control_cascade_stop(void);

/**
 * @brief 复位四电机速度环PID状态
 */
void control_speed_loop_reset(void);

void control_yaw_angle_loop_reset(void);

/**
 * @brief 复位串级控制PID状态
 */
void control_cascade_reset(void);

#endif
