#include "zf_common_headfile.h"
#ifndef _ODOMETER_H_
#define _ODOMETER_H_



/* 里程计输出数据
 * 给 position 模块读，单位都是 m。
 * strafe 左移为正，forward 前进为正。
 * travel_distance 是平面累计标量路程（不分正负） */
typedef struct
{
    float forward_distance;        /* 前后位移，单位 m，前进为正 */
    float strafe_distance;         /* 横向位移，单位 m，左移为正 */
    float travel_distance;         /* 平面累计路程，单位 m */
    float forward_velocity_mps;    /* 前后速度，单位 m/s，前进为正 */
    float strafe_velocity_mps;     /* 横向速度，单位 m/s，左移为正 */
} odometer_data_t;

extern odometer_data_t g_odometer;  /* 全局里程计，其他模块直接读 */

/* 上电调一次，内部调 odometer_reset */
void odometer_init(void);

/* 清零所有累积量和滤波器状态，调头/脱困后可手动调 */
void odometer_reset(void);

/* 100Hz 周期调用，从编码器+IMU融合算位移
 * 异常：startup_hold 期间会丢弃数据等初始化完成 */
void odometer_update_100HZ(void);

#endif
