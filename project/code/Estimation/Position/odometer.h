#include "zf_common_headfile.h"
#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#define ODOMETER_UPDATE_DT_S                (0.01f)
#define ODOMETER_FORWARD_COUNT_PER_METER    (11287.0f)
#define ODOMETER_STRAFE_COUNT_PER_METER_ABS (12100.0f)



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

/* 100Hz 周期调用：编码器速度经 yaw 解耦后积分成水平系速度和位移。 */
void odometer_update_100HZ(void);

/* 1000HZ 周期调用: 用于IMU预测*/
void odometer_update_1000HZ(void);
#endif
