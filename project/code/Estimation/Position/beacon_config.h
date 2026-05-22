#ifndef _BEACON_CONFIG_H_
#define _BEACON_CONFIG_H_

#include "zf_common_headfile.h"

typedef struct
{
    float x;                        /* 全局 X 坐标 [m]，正=右侧 */
    float y;                        /* 全局 Y 坐标 [m]，正=前方 */
} beacon_config_point_t;

void beacon_config_init(void);
void beacon_config_reset(void);
uint16 beacon_config_get_count(void);
uint8 beacon_config_get_beacon(uint16 index, beacon_config_point_t *beacon);
void beacon_config_get_initial_position(float position[2]);

#endif
