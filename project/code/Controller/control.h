/*********************************************************************************************************************
* 四电机速度环控制模块 - 头文件
*
* 功能：使用四个增量式PID实例实现四个电机速度闭环控制，PID参数由菜单统一配置
********************************************************************************************************************/

#include "zf_common_headfile.h"
#ifndef __CONTROL_H__
#define __CONTROL_H__



/* 四轮增量式PID（左前、右前、左后、右后） */
extern IncrementPID wheel_left_front_pid;
extern IncrementPID wheel_right_front_pid;
extern IncrementPID wheel_left_rear_pid;
extern IncrementPID wheel_right_rear_pid;

/* 航向角PID（外环）和角速度PID（内环） */
extern PositionalPID yaw_angle_pid;
extern PositionalPID yaw_rate_pid;

/* 串级控制中间变量（调试用） */
extern float control_yaw_angle_current;   // 当前航向角（rad）
extern float control_yaw_angle_output;    // 角度环输出（rad/s）
extern float control_yaw_rate_target;     // 角速度目标（rad/s）
extern float control_yaw_rate_current;    // 当前角速度（rad/s）
extern float control_yaw_rate_raw;        // 原始角速度（rad/s，未滤波）
extern float control_yaw_rate_avg_10ms;   // 前10ms角速度均值（rad/s）
extern float control_yaw_rate_avg_20ms;   // 前20ms角速度均值（rad/s）
extern float control_yaw_rate_output;     // 角速度环输出（编码器计数）

/* 获取当前航向角（rad，±π）
 * 数据来源：g_euler.yaw（度），取反后转弧度
 * 用途：角度环反馈、航向保持、坐标变换
 */
float control_get_current_yaw_angle(void);

/* 1kHz更新角速度滑动均值，供50Hz角速度环使用 */
void control_yaw_rate_average_update_1000HZ(void);

/* 重置航向保持状态
 * 调用时机：控制关闭时、紧急停机时
 * 效果：清除历史旋转状态，锁定当前航向为保持目标
 */
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
/* 更新航向角度环（25HZ调用）
 * yaw_angle_target: 目标航向角（rad）
 * 返回：角速度目标（rad/s），传给角速度环
 * 注意：角度归一化到±π后再算误差
 */
float control_yaw_angle_loop_update_25HZ(float yaw_angle_target);

/* 更新航向角速度环（50HZ调用）
 * yaw_rate_target: 目标角速度（rad/s）
 * 返回：旋转分量（编码器计数），用于速度环叠加
 * 数据来源：ICM42688陀螺仪（滤波后）
 */
float control_yaw_rate_loop_update_50HZ(float yaw_rate_target);

/* 更新航向保持逻辑（25HZ调用）
 * rotate_target: 旋转输入（来自遥控器）
 * 当rotate_target非零时：记录当前航向，不执行保持
 * 当rotate_target为零时：自动保持退出前的航向
 * 用途：Mode0遥控模式下，松开旋转摇杆时自动保持朝向
 */
void control_yaw_hold_update_25HZ(float rotate_target);

/* 使用角速度环输出更新四轮速度环（100HZ调用）
 * forward_target: 前后速度（编码器计数，正=前）
 * strafe_target: 左右速度（编码器计数，正=右）
 * 内部使用control_yaw_rate_output作为旋转分量
 */
void control_cascade_speed_loop_update_100HZ(float forward_target, float strafe_target);

/* 指定旋转分量更新四轮速度环（100HZ调用）
 * rotate_target: 旋转分量（编码器计数，正=顺时针）
 * 用途：可绕过角速度环直接指定旋转
 */
void control_cascade_speed_loop_update_with_rotate_100HZ(float forward_target, float strafe_target, float rotate_target);

/* 串级控制更新（50HZ调用）
 * 先更新角速度环，再触发速度环
 * 注意：实际速度环在100HZ频率单独跑，这里只更新角速度环
 */
void control_cascade_update_50HZ(float forward_target, float strafe_target, float yaw_rate_target);

/* 四轮速度环更新（100HZ调用）
 * 四个目标都是编码器周期计数
 * 麦克纳姆轮解算（外部完成）后传入，PID输出PWM控制电机
 */
void control_speed_loop_update_100HZ(float left_front_target, float right_front_target,
                                     float left_rear_target, float right_rear_target);

/* 停止速度环：复位PID + 停电机 */
void control_speed_loop_stop(void);

/* 停止串级控制：复位所有PID + 停电机 */
void control_cascade_stop(void);

/* 复位速度环PID状态（不停电机） */
void control_speed_loop_reset(void);

/* 复位角度环PID状态 + 清零输出 */
void control_yaw_angle_loop_reset(void);

/* 复位串级控制所有PID状态 + 航向保持 */
void control_cascade_reset(void);

#endif
