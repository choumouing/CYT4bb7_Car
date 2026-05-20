#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#include "zf_common_headfile.h"

#define ODOMETER_UPDATE_DT_S                (0.01f)
#define ODOMETER_FORWARD_COUNT_PER_METER    (14750.0f)
#define ODOMETER_STRAFE_COUNT_PER_METER_ABS (14000.0f)
#define ODOMETER_STATIC_ENCODER_SPEED_MPS   (0.03f)
#define ODOMETER_WHEEL_Q_THRESH             (40.0f)
#define ODOMETER_WHEEL_ROBUST_BLEND         (1.00f)
#define ODOMETER_WHEEL_GATE_HOLD_TICKS      (4U)

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
} odometer_data_t;

extern odometer_data_t g_odometer;

void odometer_init(void);
void odometer_reset(void);
void odometer_update_100HZ(void);

#endif
