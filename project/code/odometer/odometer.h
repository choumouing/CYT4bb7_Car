#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#include "zf_common_headfile.h"

typedef struct
{
    float forward_distance;        /* 前后位移，单位 m */
    float strafe_distance;         /* 左右位移，单位 m，右为正 */
    float travel_distance;         /* 平面累计路程，单位 m */
} odometer_data_t;

extern odometer_data_t g_odometer;

void odometer_init(void);
void odometer_reset(void);
void odometer_update(void);

float odometer_get_forward_distance(void);
float odometer_get_strafe_distance(void);
float odometer_get_travel_distance(void);
float odometer_get_raw_forward_distance(void);
float odometer_get_raw_strafe_distance(void);
void odometer_get_data(odometer_data_t *data);

#endif
