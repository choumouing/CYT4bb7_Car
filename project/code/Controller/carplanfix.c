#include "carplanfix.h"

#define CARPLANFIX_ROUTE_MAX_POINTS (16U)

typedef struct
{
    uint8 sequence_id;
    uint8 identified_last_beacon_id;
    uint32 identified_event_count;
    uint8 point_count;
    uint8 points[CARPLANFIX_ROUTE_MAX_POINTS];
} carplanfix_route_t;

/* 事件数为0时不参与匹配，用于无需区分识别过程的灯序。 */
static const carplanfix_route_t s_routes[] =
{
    {1U, 2U, 3U, 9U, {4U, 6U, 3U, 4U, 5U, 1U, 2U, 4U, 6U}},
    {1U, 2U, 4U, 8U, {3U, 6U, 5U, 4U, 2U, 1U, 4U, 6U}},
    {2U, 2U, 0U, 10U, {3U, 4U, 5U, 6U, 4U, 1U, 2U, 3U, 4U, 5U}},
    {3U, 6U, 4U, 8U, {3U, 2U, 1U, 4U, 6U, 5U, 4U, 2U}},
    {3U, 6U, 3U, 9U, {4U, 2U, 3U, 4U, 1U, 5U, 6U, 4U, 2U}},
    {4U, 4U, 0U, 10U, {3U, 2U, 4U, 6U, 5U, 1U, 4U, 3U, 2U, 1U}}
};

carplanfix_state_t g_carplanfix_state = {0};
static const carplanfix_route_t *s_active_route;
static uint8 s_mode3_was_active;

static const carplanfix_route_t *carplanfix_find_route(uint8 sequence_id,
                                                       uint8 last_beacon_id,
                                                       uint32 event_count)
{
    uint8 route_index;

    for(route_index = 0U;
        route_index < (uint8)(sizeof(s_routes) / sizeof(s_routes[0]));
        route_index++)
    {
        if((s_routes[route_index].sequence_id == sequence_id) &&
           (s_routes[route_index].identified_last_beacon_id == last_beacon_id) &&
           ((s_routes[route_index].identified_event_count == 0U) ||
            (s_routes[route_index].identified_event_count == event_count)))
        {
            return &s_routes[route_index];
        }
    }

    return NULL;
}

static uint8 carplanfix_route_valid(const carplanfix_route_t *route)
{
    uint8 point_index;

    if((route == NULL) ||
       (route->point_count == 0U) ||
       (route->point_count > CARPLANFIX_ROUTE_MAX_POINTS))
    {
        return 0U;
    }

    for(point_index = 0U; point_index < route->point_count; point_index++)
    {
        if((route->points[point_index] == 0U) ||
           (route->points[point_index] > BEACON_CONFIG_BEACON_COUNT))
        {
            return 0U;
        }
    }

    return 1U;
}

static void carplanfix_disable(carplanfix_disable_reason_e reason)
{
    g_carplanfix_state.status = CARPLANFIX_STATUS_DISABLED;
    g_carplanfix_state.disable_reason = (uint8)reason;
    g_carplanfix_state.correction_valid = 0U;
    g_carplanfix_state.target_event_pending = 0U;
    g_carplanfix_state.target_exit_confirm_count = 0U;
    g_carplanfix_state.mode3_beacon1_pending = 0U;
    s_active_route = NULL;
}

static void carplanfix_advance_route(void)
{
    g_carplanfix_state.target_zone_entered = 0U;
    g_carplanfix_state.target_event_pending = 0U;
    g_carplanfix_state.target_exit_confirm_count = 0U;
    g_carplanfix_state.route_index++;
    if(g_carplanfix_state.route_index >= g_carplanfix_state.route_count)
    {
        g_carplanfix_state.status = CARPLANFIX_STATUS_COMPLETE;
        g_carplanfix_state.target_beacon_id = 0U;
        g_carplanfix_state.correction_valid = 0U;
        s_active_route = NULL;
        return;
    }

    g_carplanfix_state.target_beacon_id =
        s_active_route->points[g_carplanfix_state.route_index];
}

static void carplanfix_apply_target_direction(float dx,
                                              float dy,
                                              float distance,
                                              float plan_speed,
                                              float *forward_mps,
                                              float *strafe_mps)
{
    float heading;
    float cos_heading;
    float sin_heading;
    float global_velocity_x;
    float global_velocity_y;

    global_velocity_x = dx * plan_speed / distance;
    global_velocity_y = dy * plan_speed / distance;
    heading = odometer_get_heading_rad();
    cos_heading = cosf(heading);
    sin_heading = sinf(heading);
    g_carplanfix_state.corrected_strafe_mps =
        (cos_heading * global_velocity_x) + (sin_heading * global_velocity_y);
    g_carplanfix_state.corrected_forward_mps =
        (-sin_heading * global_velocity_x) + (cos_heading * global_velocity_y);

    *forward_mps = g_carplanfix_state.corrected_forward_mps;
    *strafe_mps = g_carplanfix_state.corrected_strafe_mps;
    g_carplanfix_state.correction_valid = 1U;
}

static void carplanfix_update_route(const struct light_sequence_result *result)
{
    if(g_carplanfix_state.status == CARPLANFIX_STATUS_WAIT_SEQUENCE)
    {
        if((result->status == LIGHT_SEQUENCE_STATUS_FAILED) ||
           (result->status == LIGHT_SEQUENCE_STATUS_CONFIG_ERROR))
        {
            carplanfix_disable(CARPLANFIX_DISABLE_SEQUENCE_FAILED);
            return;
        }

        if(result->status != LIGHT_SEQUENCE_STATUS_IDENTIFIED)
        {
            return;
        }

        s_active_route = carplanfix_find_route(result->sequence_id,
                                                result->last_beacon_id,
                                                result->accepted_event_count);
        if(s_active_route == NULL)
        {
            carplanfix_disable(CARPLANFIX_DISABLE_ROUTE_NOT_FOUND);
            return;
        }
        if(carplanfix_route_valid(s_active_route) == 0U)
        {
            carplanfix_disable(CARPLANFIX_DISABLE_INVALID_ROUTE);
            return;
        }

        g_carplanfix_state.status = CARPLANFIX_STATUS_TRACKING;
        g_carplanfix_state.sequence_id = result->sequence_id;
        g_carplanfix_state.identified_last_beacon_id = result->last_beacon_id;
        g_carplanfix_state.route_index = 0U;
        g_carplanfix_state.route_count = s_active_route->point_count;
        g_carplanfix_state.target_beacon_id = s_active_route->points[0];
        g_carplanfix_state.last_event_count = result->accepted_event_count;
        g_carplanfix_state.target_zone_entered = 0U;
        g_carplanfix_state.target_event_pending = 0U;
        g_carplanfix_state.target_exit_confirm_count = 0U;
        return;
    }

    if(g_carplanfix_state.status != CARPLANFIX_STATUS_TRACKING)
    {
        return;
    }

    if((result->status != LIGHT_SEQUENCE_STATUS_IDENTIFIED) ||
       (result->sequence_id != g_carplanfix_state.sequence_id))
    {
        carplanfix_disable(CARPLANFIX_DISABLE_SEQUENCE_FAILED);
        return;
    }

    if(result->accepted_event_count == g_carplanfix_state.last_event_count)
    {
        return;
    }

    g_carplanfix_state.last_event_count = result->accepted_event_count;
    if(result->last_beacon_id != g_carplanfix_state.target_beacon_id)
    {
        carplanfix_disable(CARPLANFIX_DISABLE_UNEXPECTED_BEACON);
        return;
    }

    g_carplanfix_state.target_event_pending = 1U;
}

void carplanfix_reset(void)
{
    g_carplanfix_state = (carplanfix_state_t){0};
    g_carplanfix_state.status = CARPLANFIX_STATUS_WAIT_SEQUENCE;
    s_active_route = NULL;
    s_mode3_was_active = 0U;
}

uint8 carplanfix_resolve(const struct light_sequence_result *light_sequence_result,
                         uint8 mode3_active,
                         uint8 air_plan_valid,
                         float air_forward_mps,
                         float air_strafe_mps,
                         float *forward_mps,
                         float *strafe_mps)
{
    beacon_config_data_t map_data;
    beacon_config_point_t target;
    float plan_speed;
    float dx;
    float dy;
    float distance;

    if((light_sequence_result == NULL) ||
       (forward_mps == NULL) ||
       (strafe_mps == NULL))
    {
        return 0U;
    }

    *forward_mps = air_forward_mps;
    *strafe_mps = air_strafe_mps;
    g_carplanfix_state.correction_valid = 0U;
    g_carplanfix_state.near_beacon = 0U;
    plan_speed = sqrtf((air_forward_mps * air_forward_mps) +
                       (air_strafe_mps * air_strafe_mps));
    g_carplanfix_state.target_speed_mps = plan_speed;

    carplanfix_update_route(light_sequence_result);

    mode3_active = (mode3_active != 0U) ? 1U : 0U;
    if((mode3_active != 0U) && (s_mode3_was_active == 0U))
    {
        g_carplanfix_state.mode3_beacon1_pending = 1U;
    }
    else if(mode3_active == 0U)
    {
        g_carplanfix_state.mode3_beacon1_pending = 0U;
    }
    s_mode3_was_active = mode3_active;

    if(g_carplanfix_state.status == CARPLANFIX_STATUS_DISABLED)
    {
        g_carplanfix_state.mode3_beacon1_pending = 0U;
        return 0U;
    }

    if(g_carplanfix_state.mode3_beacon1_pending != 0U)
    {
        beacon_config_get_predata(&map_data);
        target = map_data.beacons[0];
        dx = target.x - g_odometer.position[x];
        dy = target.y - g_odometer.position[y];
        distance = sqrtf((dx * dx) + (dy * dy));
        g_carplanfix_state.target_distance_m = distance;
        g_carplanfix_state.target_position[x] = target.x;
        g_carplanfix_state.target_position[y] = target.y;
        g_carplanfix_state.near_beacon =
            (distance <= CARPLANFIX_TARGET_RADIUS_M) ? 1U : 0U;

        if(distance <= CARPLANFIX_TARGET_RADIUS_M)
        {
            g_carplanfix_state.mode3_beacon1_pending = 0U;
        }
        else
        {
            if((air_plan_valid != 0U) &&
               (plan_speed >= CARPLANFIX_MIN_PLAN_SPEED_MPS))
            {
                carplanfix_apply_target_direction(dx,
                                                  dy,
                                                  distance,
                                                  plan_speed,
                                                  forward_mps,
                                                  strafe_mps);
                return 1U;
            }
            return 0U;
        }
    }

    if(g_carplanfix_state.status != CARPLANFIX_STATUS_TRACKING)
    {
        return 0U;
    }

    beacon_config_get_predata(&map_data);
    target = map_data.beacons[g_carplanfix_state.target_beacon_id - 1U];
    dx = target.x - g_odometer.position[x];
    dy = target.y - g_odometer.position[y];
    distance = sqrtf((dx * dx) + (dy * dy));
    g_carplanfix_state.target_distance_m = distance;
    g_carplanfix_state.target_position[x] = target.x;
    g_carplanfix_state.target_position[y] = target.y;
    g_carplanfix_state.near_beacon =
        (distance <= CARPLANFIX_TARGET_RADIUS_M) ? 1U : 0U;

    if(g_carplanfix_state.target_event_pending != 0U)
    {
        if(distance <= CARPLANFIX_TARGET_RADIUS_M)
        {
            g_carplanfix_state.target_zone_entered = 1U;
            g_carplanfix_state.target_exit_confirm_count = 0U;
            return 0U;
        }

        if(g_carplanfix_state.target_exit_confirm_count <
           CARPLANFIX_TARGET_EXIT_CONFIRM_CYCLES)
        {
            g_carplanfix_state.target_exit_confirm_count++;
        }
        if(g_carplanfix_state.target_exit_confirm_count >=
           CARPLANFIX_TARGET_EXIT_CONFIRM_CYCLES)
        {
            carplanfix_advance_route();
        }
        return 0U;
    }

    if(g_carplanfix_state.target_zone_entered != 0U)
    {
        if(distance > CARPLANFIX_TARGET_NO_EVENT_EXIT_RADIUS_M)
        {
            carplanfix_disable(CARPLANFIX_DISABLE_LEFT_TARGET_WITHOUT_EVENT);
        }
        return 0U;
    }

    if(distance <= CARPLANFIX_TARGET_RADIUS_M)
    {
        g_carplanfix_state.target_zone_entered = 1U;
        return 0U;
    }

    if((air_plan_valid == 0U) ||
       (plan_speed < CARPLANFIX_MIN_PLAN_SPEED_MPS))
    {
        return 0U;
    }

    carplanfix_apply_target_direction(dx,
                                      dy,
                                      distance,
                                      plan_speed,
                                      forward_mps,
                                      strafe_mps);

    return 1U;
}
