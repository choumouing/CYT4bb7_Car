/*********************************************************************************************************************
* 菜单用户配置头文件 - 参数集中管理
* 功能：定义菜单结构和参数配置，导出所有菜单可调参数
* 版本：v3.1 (多航点导航系统 + 校准表参数)
* 日期：2025-12-15
********************************************************************************************************************/

#ifndef _MENU_CONFIG_H_
#define _MENU_CONFIG_H_

#include "menu_core.h"
#include "zf_common_headfile.h"

//====================================================参数声明====================================================

// S曲线速度规划参数
extern float s_curve_v_max;            // 最大速度 (mm/s)
extern float s_curve_a_max;            // 最大加速度 (mm/s²)
extern float s_curve_j_max;            // 最大Jerk (mm/s³)

// S曲线算法参数
extern float s_curve_max_iter;         // vlim迭代最大次数
extern float s_curve_conv_tol;         // 位移收敛容差 (mm)
extern float s_curve_min_dist;         // 极短距离阈值 (mm)

// 导航任务参数
extern float nav_dist_tol;             // 到达目标距离容差 (mm)

// 航向角PID参数
extern float yaw_kp_2;                 // 二次项比例系数 [(rad/s)/°²]
extern float yaw_kp_1;                 // 一次项比例系数 [(rad/s)/°]
extern float yaw_ki;                   // 积分系数 [(rad/s)/°]
extern float yaw_kd;                   // 微分系数 [(rad/s)/°]
extern float yaw_i_limit;              // 积分限幅 (°)
extern float yaw_output_limit;         // 输出限幅 (rad/s)
extern float yaw_pos_d_lpf;            // 微分低通滤波系数 [0-1]
extern float yaw_tolerance;            // 航向角容差 (°)

// Yaw角速度PID参数
extern float yaw_rate_kp;              // 比例系数 [(pulse/5ms)/(rad/s)]
extern float yaw_rate_ki;              // 积分系数 [(pulse/5ms)/(rad/s)]
extern float yaw_rate_kd;              // 微分系数 [(pulse/5ms)/(rad/s)]
extern float yaw_rate_i_limit;         // 积分限幅 (rad/s)
extern float yaw_rate_output_limit;    // 输出限幅 (pulse/5ms)
extern float yaw_rate_d_lpf;           // 微分低通滤波系数 [0-1]

// 轮速PID参数（增量式）
extern float wheel_kp;                 // 比例系数
extern float wheel_ki;                 // 积分系数
extern float wheel_kd;                 // 微分系数
extern float wheel_output_limit;       // 输出限幅 (PWM)
extern float wheel_inc_i_lpf;          // 积分低通滤波系数

// UWB位置环X方向PID参数（标签右向相对误差 -> 横移速度）
extern float uwb_x_kp;
extern float uwb_x_ki;
extern float uwb_x_kd;
extern float uwb_x_i_limit;
extern float uwb_x_output_limit;
extern float uwb_x_d_lpf;

// UWB位置环Y方向PID参数（标签前向相对误差 -> 前进速度）
extern float uwb_y_kp;
extern float uwb_y_ki;
extern float uwb_y_kd;
extern float uwb_y_i_limit;
extern float uwb_y_output_limit;
extern float uwb_y_d_lpf;

// 发车标志
extern float is_car_running;           // 发车状态 (0:停止 1:运行)

// X方向校准表参数（5个标定点的PULSE_PER_MM）
extern float calib_x_1000;             // 1000mm标定点
extern float calib_x_2000;             // 2000mm标定点
extern float calib_x_3000;             // 3000mm标定点
extern float calib_x_4000;             // 4000mm标定点
extern float calib_x_5000;             // 5000mm标定点

// Y方向校准表参数（5个标定点的PULSE_PER_MM）
extern float calib_y_1000;             // 1000mm标定点
extern float calib_y_2000;             // 2000mm标定点
extern float calib_y_3000;             // 3000mm标定点
extern float calib_y_4000;             // 4000mm标定点
extern float calib_y_5000;             // 5000mm标定点

// 航点参数（10个航点，每个4个参数：x, y, yaw, enabled）
extern float wp0_x, wp0_y, wp0_yaw, wp0_en;
extern float wp1_x, wp1_y, wp1_yaw, wp1_en;
extern float wp2_x, wp2_y, wp2_yaw, wp2_en;
extern float wp3_x, wp3_y, wp3_yaw, wp3_en;
extern float wp4_x, wp4_y, wp4_yaw, wp4_en;
extern float wp5_x, wp5_y, wp5_yaw, wp5_en;
extern float wp6_x, wp6_y, wp6_yaw, wp6_en;
extern float wp7_x, wp7_y, wp7_yaw, wp7_en;
extern float wp8_x, wp8_y, wp8_yaw, wp8_en;
extern float wp9_x, wp9_y, wp9_yaw, wp9_en;

//====================================================用户配置接口====================================================
void menu_config_init(void);           // 初始化菜单配置（注册参数和设置根菜单）

#endif
