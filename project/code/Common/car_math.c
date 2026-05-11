#include "car_math.h"


float car_math_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

float car_math_minf(float a, float b)
{
    return (a < b) ? a : b;
}

float car_math_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

float car_math_clampf(float value, float min_value, float max_value)
{
    if(value < min_value)
    {
        return min_value;
    }

    if(value > max_value)
    {
        return max_value;
    }

    return value;
}

float car_math_limit_absf(float value, float limit)
{
    if(value > limit)
    {
        return limit;
    }

    if(value < -limit)
    {
        return -limit;
    }

    return value;
}

float car_math_deadband(float value, float deadband)
{
    if(car_math_absf(value) <= deadband)
    {
        return 0.0f;
    }

    return value;
}

float car_math_soft_deadband(float value, float deadband)
{
    float magnitude;

    magnitude = car_math_absf(value) - deadband;
    if(magnitude <= 0.0f)
    {
        return 0.0f;
    }

    return (value >= 0.0f) ? magnitude : -magnitude;
}

float car_math_map_linear(float value,
                          float in_min,
                          float in_max,
                          float out_min,
                          float out_max)
{
    if(in_max == in_min)
    {
        return out_min;
    }

    return out_min + ((value - in_min) * (out_max - out_min) / (in_max - in_min));
}
