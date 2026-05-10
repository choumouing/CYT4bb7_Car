#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#include "zf_common_headfile.h"

typedef struct
{
    float forward_distance;        /* 前后位移，单位 m，前进为正 */
    float strafe_distance;         /* 横向位移，单位 m，左移为正 */
    float travel_distance;         /* 平面累计路程，单位 m */
} odometer_data_t;

extern odometer_data_t g_odometer;

void odometer_init(void);
void odometer_reset(void);
void odometer_update(void);

#endif
