#ifndef CAR_FILTER_H
#define CAR_FILTER_H

#include "zf_common_headfile.h"

typedef struct
{
    float value;
    uint8 ready;
} car_filter_lpf1_t;

void car_filter_lpf1_reset(car_filter_lpf1_t *filter, float value);
float car_filter_lpf1_update(car_filter_lpf1_t *filter, float input, float alpha);
float car_filter_lpf1_apply(float previous, float input, float dt_s, float tau_s);
float car_filter_median3f(float a, float b, float c);

#endif /* CAR_FILTER_H */
