/*
 * 模块职责：主循环调度器。
 * PIT 中断只置标志位，car_loop_poll() 在主循环中清标志并执行周期任务。
 *
 * 调用节奏：
 *   car_loop_init() 在系统启动时调用一次。
 *   car_loop_poll() 在 while(1) 主循环中轮询，内部按 1000/100/25 Hz 分频执行。
 *
 * 数据来源：
 *   空地串口接收无人机运行数据和遥控通道。
 *   遥控通道来自无人机 CRSF 转发的 ch0~ch7。
 */

#ifndef CAR_LOOP_H
#define CAR_LOOP_H

#include "zf_common_headfile.h"

/* 100Hz 周期任务标志，PIT 中断置 1，poll 中清 0 */
extern volatile uint8_t timer_100HZ_flag;
/* 25Hz 周期任务标志 */
extern volatile uint8_t timer_25HZ_flag;
/* 1000Hz 节拍计数，用于超时判断和在线状态老化 */
extern volatile uint16 g_tick_1000HZ;
/* 1ms tick 累计计数 */
extern volatile uint32 tick_1000us_cnt;

/* 前进速度目标，单位 m/s，正方向为车头朝向 */
extern float car_forward_target;
/* 横移速度目标，单位 m/s，正方向为车身右侧 */
extern float car_strafe_target;
/* 控制使能标志：1=允许电机输出，0=停止输出 */
extern uint8 car_control_enabled;
/* 急停激活标志：1=急停生效，电机强制停止 */
extern uint8 car_emergency_stop_active;

/* 遥控通道标准化值（来自无人机 CRSF 转发） */
/* 范围 -1.0 ~ +1.0，ch0=油门/前后，ch1=横移，ch2=航向，ch3=辅助等 */
extern volatile float g_air_tof_fused_height_mm;
extern volatile float g_air_euler_roll;
extern volatile float g_air_euler_pitch;
extern volatile float g_air_euler_yaw;
extern volatile float g_air_pos_est_vel_x;
extern volatile float g_air_pos_est_vel_y;
extern volatile float g_air_state;

extern volatile float g_air_crsf_std_ch0;
extern volatile float g_air_crsf_std_ch1;
extern volatile float g_air_crsf_std_ch2;
extern volatile float g_air_crsf_std_ch3;
extern volatile float g_air_crsf_std_ch4;
extern volatile float g_air_crsf_std_ch5;
extern volatile float g_air_crsf_std_ch6;
extern volatile float g_air_crsf_std_ch7;
extern volatile float g_air_yaw_angle_target_deg;

/* 初始化：清零所有状态和标志位 */
void car_loop_init(void);
/* 主循环轮询：内部按 1000/100/25Hz 分频执行各周期任务 */
void car_loop_poll(void);

#endif
