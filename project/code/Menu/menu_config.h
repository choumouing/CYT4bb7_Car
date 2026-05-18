/*********************************************************************************************************************
* 菜单用户配置头文件 - 参数集中管理
*
* 功能：集中管理四电机速度环参数，参数支持菜单调节和Flash存档
********************************************************************************************************************/

#include "zf_common_headfile.h"
#ifndef _MENU_CONFIG_H_
#define _MENU_CONFIG_H_



//====================================================速度环参数声明====================================================
extern float wheel_kp;                 // 轮速环比例系数
extern float wheel_ki;                 // 轮速环积分系数
extern float wheel_kd;                 // 轮速环微分系数
extern float wheel_output_limit;       // 轮速环输出限幅 (PWM)
extern float wheel_i_limit;            // 轮速环积分限幅

//====================================================用户配置接口====================================================
void menu_config_init(void);           // 初始化菜单配置（注册参数和设置根菜单）

extern float yaw_angle_kp;
extern float yaw_angle_ki;
extern float yaw_angle_kd;
extern float yaw_angle_i_limit;
extern float yaw_angle_output_limit;

extern float yaw_rate_kp;
extern float yaw_rate_ki;
extern float yaw_rate_kd;
extern float yaw_rate_i_limit;
extern float yaw_rate_output_limit;

extern float uwb_follow_deadband_x_cm;
extern float uwb_follow_deadband_y_cm;
extern float uwb_follow_output_limit;
extern float uwb_follow_i_limit;
extern float uwb_follow_x_kp;
extern float uwb_follow_x_ki;
extern float uwb_follow_x_kd;
extern float uwb_follow_y_kp;
extern float uwb_follow_y_ki;
extern float uwb_follow_y_kd;

extern float s_curve_max_iter;
extern float s_curve_conv_tol;
extern float s_curve_min_dist;

#endif
