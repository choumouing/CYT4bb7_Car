#include "carplanfix.h"

#define CARPLANFIX_DEG_TO_RAD        (0.017453292519943295f)
#define CARPLANFIX_PI                (3.14159265358979323846f)
#define CARPLANFIX_TWO_PI            (6.28318530717958647692f)
#define CARPLANFIX_RAD_TO_DEG        (57.295779513082320876f)
#define CARPLANFIX_INVALID_ANGLE_DEG (360.0f)

carplanfix_state_t g_carplanfix_state = {0};

static float carplanfix_wrap_pi(float angle_rad)
{
    while(angle_rad > CARPLANFIX_PI)
    {
        angle_rad -= CARPLANFIX_TWO_PI;
    }

    while(angle_rad < -CARPLANFIX_PI)
    {
        angle_rad += CARPLANFIX_TWO_PI;
    }

    return angle_rad;
}

static void carplanfix_set_fallback(float air_forward_mps,
                                    float air_strafe_mps,
                                    float *forward_mps,
                                    float *strafe_mps)
{
    *forward_mps = air_forward_mps;
    *strafe_mps = air_strafe_mps;
}

static uint8 carplanfix_find_beacon(float direction_x,
                                    float direction_y,
                                    beacon_config_point_t *target,
                                    uint16 *target_index,
                                    float *best_angle,
                                    float *second_angle)
{
    beacon_config_data_t predata;
    uint16 index;
    uint8 found = 0U;
    uint16 matched_count = 0U;

    *target_index = CARPLANFIX_INVALID_BEACON_INDEX;
    *best_angle = CARPLANFIX_INVALID_ANGLE_DEG;
    *second_angle = CARPLANFIX_INVALID_ANGLE_DEG;
    beacon_config_get_predata(&predata);

    for(index = 0U; index < BEACON_CONFIG_BEACON_COUNT; index++)
    {
        beacon_config_point_t beacon = predata.beacons[index];
        float dx;
        float dy;
        float distance;
        float along;
        float cross;
        float angle_error;

        dx = beacon.x - g_odometer.position[x];
        dy = beacon.y - g_odometer.position[y];
        distance = sqrtf((dx * dx) + (dy * dy));
        if(distance < CARPLANFIX_NEAREST_REJECT_DISTANCE_M)
        {
            return 0U;
        }

        along = (direction_x * dx) + (direction_y * dy);
        if(along <= 0.0f)
        {
            continue;
        }

        cross = (direction_x * dy) - (direction_y * dx);
        angle_error = fabsf(atan2f(cross, along)) * CARPLANFIX_RAD_TO_DEG;
        if(angle_error <= CARPLANFIX_MAX_ANGLE_ERROR_DEG)
        {
            matched_count++;
        }

        if(angle_error < *best_angle)
        {
            *second_angle = *best_angle;
            *best_angle = angle_error;
            *target_index = index;
            *target = beacon;
            found = 1U;
        }
        else if(angle_error < *second_angle)
        {
            *second_angle = angle_error;
        }
    }

    if((found == 0U) || (matched_count != 1U))
    {
        return 0U;
    }

    return 1U;
}

void carplanfix_reset(void)
{
    g_carplanfix_state = (carplanfix_state_t){0};
    g_carplanfix_state.beacon_index = CARPLANFIX_INVALID_BEACON_INDEX;
    g_carplanfix_state.angle_error_deg = CARPLANFIX_INVALID_ANGLE_DEG;
    g_carplanfix_state.second_angle_error_deg = CARPLANFIX_INVALID_ANGLE_DEG;
}

uint8 carplanfix_resolve(float air_forward_mps,
                         float air_strafe_mps,
                         float air_yaw_target_deg,
                         float *forward_mps,
                         float *strafe_mps)
{
    beacon_config_point_t target;
    uint16 target_index;
    float best_angle;
    float second_angle;
    float plan_speed;
    float current_heading;
    float target_heading;
    float cos_heading;
    float sin_heading;
    float direction_x;
    float direction_y;
    float dx;
    float dy;
    float distance;
    float global_velocity_x;
    float global_velocity_y;

    if((forward_mps == NULL) || (strafe_mps == NULL))
    {
        return 0U;
    }

    carplanfix_reset();
    carplanfix_set_fallback(air_forward_mps,
                            air_strafe_mps,
                            forward_mps,
                            strafe_mps);

    plan_speed = sqrtf((air_forward_mps * air_forward_mps) +
                       (air_strafe_mps * air_strafe_mps));
    g_carplanfix_state.target_speed_mps = plan_speed;
    if(plan_speed < CARPLANFIX_MIN_PLAN_SPEED_MPS)
    {
        return 0U;
    }

    /*
     * 里程计航向以复位时车头为全局Y正方向。无人机与车辆yaw控制均采用
     * 同一绝对零点，因此可用当前yaw与目标yaw之差得到目标相对航向。
     */
    current_heading = odometer_get_heading_rad();
    target_heading = current_heading +
                     ((g_euler.yaw - air_yaw_target_deg) * CARPLANFIX_DEG_TO_RAD);
    target_heading = carplanfix_wrap_pi(target_heading);
    cos_heading = cosf(target_heading);
    sin_heading = sinf(target_heading);

    direction_x = ((cos_heading * air_strafe_mps) -
                   (sin_heading * air_forward_mps)) / plan_speed;
    direction_y = ((sin_heading * air_strafe_mps) +
                   (cos_heading * air_forward_mps)) / plan_speed;

    if(carplanfix_find_beacon(direction_x,
                              direction_y,
                              &target,
                              &target_index,
                              &best_angle,
                              &second_angle) == 0U)
    {
        return 0U;
    }

    dx = target.x - g_odometer.position[x];
    dy = target.y - g_odometer.position[y];
    distance = sqrtf((dx * dx) + (dy * dy));
    if(distance <= CARPLANFIX_APPLY_MIN_DISTANCE_M)
    {
        return 0U;
    }

    g_carplanfix_state.matched = 1U;
    g_carplanfix_state.beacon_index = target_index;
    g_carplanfix_state.angle_error_deg = best_angle;
    g_carplanfix_state.second_angle_error_deg = second_angle;
    g_carplanfix_state.target_distance_m = distance;
    g_carplanfix_state.target_position[x] = target.x;
    g_carplanfix_state.target_position[y] = target.y;

    global_velocity_x = dx * plan_speed / distance;
    global_velocity_y = dy * plan_speed / distance;

    /* 使用车辆当前实际航向，将全局直指信标的速度转换回车体系。 */
    cos_heading = cosf(current_heading);
    sin_heading = sinf(current_heading);
    g_carplanfix_state.corrected_strafe_mps =
        (cos_heading * global_velocity_x) + (sin_heading * global_velocity_y);
    g_carplanfix_state.corrected_forward_mps =
        (-sin_heading * global_velocity_x) + (cos_heading * global_velocity_y);

    *forward_mps = g_carplanfix_state.corrected_forward_mps;
    *strafe_mps = g_carplanfix_state.corrected_strafe_mps;
    g_carplanfix_state.correction_valid = 1U;

    return 1U;
}
