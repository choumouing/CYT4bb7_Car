#ifndef SBUS_H
#define SBUS_H

#include "zf_common_headfile.h"

#define SBUS_CHANNEL_COUNT              (10U)
#define SBUS_USED_CHANNEL_COUNT         (6U)

#define SBUS_STD_AXIS_MIN               (-1000)
#define SBUS_STD_AXIS_MAX               (1000)
#define SBUS_STD_SWITCH_UP              (0)
#define SBUS_STD_SWITCH_DOWN            (1)
#define SBUS_STD_SWITCH_INVALID         (-1)

#define SBUS_CH1                        (0U)
#define SBUS_CH2                        (1U)
#define SBUS_CH3                        (2U)
#define SBUS_CH4                        (3U)
#define SBUS_CH5                        (4U)
#define SBUS_CH6                        (5U)

typedef struct
{
    uint16 raw_channel[SBUS_CHANNEL_COUNT];
    int16 std_channel[SBUS_CHANNEL_COUNT];
    uint8 channel_valid[SBUS_CHANNEL_COUNT];
    uint8 receiver_online;
    uint8 frame_updated;
} sbus_state_t;

extern sbus_state_t g_sbus_state;

void sbus_init(void);
void sbus_update_25HZ(void);
const sbus_state_t *sbus_get_state(void);

#endif /* SBUS_H */
