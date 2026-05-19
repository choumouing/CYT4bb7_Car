#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#include "zf_common_headfile.h"

#define ODOMETER_UPDATE_DT_S                (0.01f)
#define ODOMETER_IMU_UPDATE_DT_S            (0.001f)
#define ODOMETER_FORWARD_COUNT_PER_METER    (11287.0f)
#define ODOMETER_STRAFE_COUNT_PER_METER_ABS (12100.0f)
#define ODOMETER_ENCODER_BLEND_ALPHA        (0.024f)
#define ODOMETER_VELOCITY_LEAK_TAU_S        (0.035f)
#define ODOMETER_VELOCITY_LEAK_FACTOR       (0.9718329f)
#define ODOMETER_STATIC_ENCODER_SPEED_MPS   (0.03f)
#define ODOMETER_STATIC_ACCEL_MPS2          (0.25f)
#define ODOMETER_ACCEL_BIAS_ALPHA_STATIC    (0.02f)

enum
{
    x = 0,
    y = 1,
    ODOMETER_X = x,
    ODOMETER_Y = y,
    ODOMETER_AXIS_NUM = 2
};

typedef struct
{
    float position[ODOMETER_AXIS_NUM];
    float vel[ODOMETER_AXIS_NUM];
    float acc[ODOMETER_AXIS_NUM];
} odometer_data_t;

extern odometer_data_t g_odometer;

void odometer_init(void);
void odometer_reset(void);
void odometer_update_100HZ(void);
void odometer_update_1000HZ(void);

#endif
