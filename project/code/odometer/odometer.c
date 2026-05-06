#include "odometer.h"
#include "../encoder/encoder_control.h"
#include <math.h>

odometer_data_t g_odometer = {0.0f, 0.0f, 0.0f};

void odometer_init(void)
{
    odometer_reset();
}

void odometer_reset(void)
{
    g_odometer.forward_distance = 0.0f;
    g_odometer.strafe_distance = 0.0f;
    g_odometer.travel_distance = 0.0f;
}

void odometer_update(void)
{
    float left_front = encoder_get_left_front_filtered_count();
    float right_front = encoder_get_right_front_filtered_count();
    float left_rear = encoder_get_left_rear_filtered_count();
    float right_rear = encoder_get_right_rear_filtered_count();
    float forward_delta;
    float strafe_delta;

    forward_delta = (left_front + right_front + left_rear + right_rear) * 0.25f;
    strafe_delta = (-left_front + right_front + left_rear - right_rear) * 0.25f;

    forward_delta *= ODOMETER_DISTANCE_PER_COUNT;
    strafe_delta *= ODOMETER_DISTANCE_PER_COUNT;

    g_odometer.forward_distance += forward_delta;
    g_odometer.strafe_distance += strafe_delta;
    g_odometer.travel_distance += sqrtf((forward_delta * forward_delta) +
                                        (strafe_delta * strafe_delta));
}

float odometer_get_forward_distance(void)
{
    return g_odometer.forward_distance;
}

float odometer_get_strafe_distance(void)
{
    return g_odometer.strafe_distance;
}

float odometer_get_travel_distance(void)
{
    return g_odometer.travel_distance;
}

void odometer_get_data(odometer_data_t *data)
{
    if(NULL == data)
    {
        return;
    }

    *data = g_odometer;
}
