#ifndef CAR_MATH_H
#define CAR_MATH_H

#include "zf_common_headfile.h"

float car_math_absf(float value);
float car_math_minf(float a, float b);
float car_math_maxf(float a, float b);
float car_math_clampf(float value, float min_value, float max_value);
float car_math_limit_absf(float value, float limit);
float car_math_deadband(float value, float deadband);
float car_math_soft_deadband(float value, float deadband);
float car_math_map_linear(float value,
                          float in_min,
                          float in_max,
                          float out_min,
                          float out_max);

#endif /* CAR_MATH_H */
