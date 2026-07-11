#ifndef _BEACON_POSITION_RECORDER_H_
#define _BEACON_POSITION_RECORDER_H_

#include "zf_common_headfile.h"

#define BEACON_POSITION_RECORDER_MAX_POINTS (16U)

typedef struct
{
    float position[ODOMETER_AXIS_NUM];
    float points[BEACON_POSITION_RECORDER_MAX_POINTS][ODOMETER_AXIS_NUM];
    uint16 point_count;
    uint8 active;
    uint8 full;
} beacon_position_recorder_t;

extern beacon_position_recorder_t g_beacon_position_recorder;

/* 上电初始化，记录器默认不工作。 */
void beacon_position_recorder_init(void);

/* 进入记录模式：清空上一轮结果，并把当前位置定义为全局坐标 (0, 0)。 */
void beacon_position_recorder_enter(void);

/* 退出记录模式：停止更新，但保留已经记录的坐标。 */
void beacon_position_recorder_exit(void);

/* 100Hz 更新：按 odometer 全局速度积分，并在 Air CRSF CH8 上升沿记录坐标。 */
void beacon_position_recorder_update_100HZ(void);

#endif
