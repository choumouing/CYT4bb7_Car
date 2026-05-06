#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#include "zf_common_headfile.h"

/*
 * 里程计距离换算系数。
 * 默认 1.0f 表示输出单位为编码器计数；完成轮径、减速比、编码器线数标定后可改为 mm/count。
 */
#define ODOMETER_DISTANCE_PER_COUNT        (1.0f)

typedef struct
{
    float forward_distance;        /* 相对上电位置的前后方向位移 */
    float strafe_distance;         /* 相对上电位置的左右方向位移 */
    float travel_distance;         /* 车体平面路程累计值 */
} odometer_data_t;

extern odometer_data_t g_odometer;

void odometer_init(void);
void odometer_reset(void);
void odometer_update(void);

float odometer_get_forward_distance(void);
float odometer_get_strafe_distance(void);
float odometer_get_travel_distance(void);
void odometer_get_data(odometer_data_t *data);

#endif
