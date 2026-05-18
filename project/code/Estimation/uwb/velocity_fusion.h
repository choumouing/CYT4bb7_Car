#ifndef VELOCITY_FUSION_H
#define VELOCITY_FUSION_H

#include "zf_common_headfile.h"

typedef struct
{
    float pos_right_cm;
    float pos_forward_cm;
    float vel_right_cmps;
    float vel_forward_cmps;
    float air_right_cmps;
    float air_forward_cmps;
    float car_right_cmps;
    float car_forward_cmps;
    float uwb_right_cm;
    float uwb_forward_cm;
    float residual_right_cm;
    float residual_forward_cm;
    uint8 valid;
    uint8 uwb_updated;
} velocity_fusion_state_t;

void velocity_fusion_init(void);
void velocity_fusion_reset(void);
void velocity_fusion_update_100HZ(uint32 now_ms);
uint8 velocity_fusion_get_state(velocity_fusion_state_t *state);

#endif /* VELOCITY_FUSION_H */
