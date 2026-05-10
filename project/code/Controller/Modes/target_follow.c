#include "target_follow.h"


target_follow_state_t g_target_follow_state = {0};

static PositionalPID s_target_forward_pid;
static PositionalPID s_target_strafe_pid;

static float target_follow_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float target_follow_limit(float value, float limit)
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

static float target_follow_deadband(float value, float deadband)
{
    if(target_follow_absf(value) <= deadband)
    {
        return 0.0f;
    }

    return value;
}

static float target_follow_distance(float strafe_a, float forward_a,
                                    float strafe_b, float forward_b)
{
    float ds;
    float df;

    ds = strafe_a - strafe_b;
    df = forward_a - forward_b;
    return sqrtf((ds * ds) + (df * df));
}

static void target_follow_pid_init(void)
{
    PositionalPID_Init(&s_target_forward_pid,
                       0.0f,
                       TARGET_FOLLOW_POS_KP,
                       TARGET_FOLLOW_POS_KI,
                       TARGET_FOLLOW_POS_KD,
                       TARGET_FOLLOW_POS_I_LIMIT,
                       TARGET_FOLLOW_OUTPUT_LIMIT);
    PositionalPID_Init(&s_target_strafe_pid,
                       0.0f,
                       TARGET_FOLLOW_POS_KP,
                       TARGET_FOLLOW_POS_KI,
                       TARGET_FOLLOW_POS_KD,
                       TARGET_FOLLOW_POS_I_LIMIT,
                       TARGET_FOLLOW_OUTPUT_LIMIT);
}

static void target_follow_clear_runtime(void)
{
    g_target_follow_state.car_strafe_m = 0.0f;
    g_target_follow_state.car_forward_m = 0.0f;
    g_target_follow_state.tag_strafe_m = 0.0f;
    g_target_follow_state.tag_forward_m = 0.0f;
    g_target_follow_state.tag_relative_strafe_m = 0.0f;
    g_target_follow_state.tag_relative_forward_m = 0.0f;
    g_target_follow_state.car_tag_distance_m = 0.0f;
    g_target_follow_state.target_tag_distance_m = 0.0f;
    g_target_follow_state.target_car_distance_m = 0.0f;
    g_target_follow_state.target_error_strafe_m = 0.0f;
    g_target_follow_state.target_error_forward_m = 0.0f;
    g_target_follow_state.target_pid_strafe_output = 0.0f;
    g_target_follow_state.target_pid_forward_output = 0.0f;
    g_target_follow_state.forward_target = 0.0f;
    g_target_follow_state.strafe_target = 0.0f;
    g_target_follow_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
    g_target_follow_state.candidate_index = TARGET_FOLLOW_INVALID_INDEX;
    g_target_follow_state.mode = TARGET_FOLLOW_MODE_IDLE;
    g_target_follow_state.tag_online = 0U;
    g_target_follow_state.car_in_tag_range = 0U;
    g_target_follow_state.target_in_tag_range = 0U;
    g_target_follow_state.output_valid = 0U;
}

static void target_follow_apply_vector_limit(void)
{
    float abs_forward;
    float abs_strafe;
    float max_value;
    float scale;

    abs_forward = target_follow_absf(g_target_follow_state.forward_target);
    abs_strafe = target_follow_absf(g_target_follow_state.strafe_target);
    max_value = (abs_forward > abs_strafe) ? abs_forward : abs_strafe;

    if(max_value <= TARGET_FOLLOW_OUTPUT_LIMIT)
    {
        return;
    }

    scale = TARGET_FOLLOW_OUTPUT_LIMIT / max_value;
    g_target_follow_state.forward_target *= scale;
    g_target_follow_state.strafe_target *= scale;
}

static void target_follow_copy_tag_follow_output(void)
{
    g_target_follow_state.mode = TARGET_FOLLOW_MODE_FOLLOW_TAG;
    g_target_follow_state.output_valid = g_uwb_follow_state.output_valid;
    g_target_follow_state.target_pid_forward_output = 0.0f;
    g_target_follow_state.target_pid_strafe_output = 0.0f;
    g_target_follow_state.forward_target = (0U != g_uwb_follow_state.output_valid) ?
                                           g_uwb_follow_state.forward_target : 0.0f;
    g_target_follow_state.strafe_target = (0U != g_uwb_follow_state.output_valid) ?
                                          g_uwb_follow_state.strafe_target : 0.0f;
}

static uint8 target_follow_target_is_eligible(uint8 index, float *distance_m)
{
    float distance;

    if((index >= TARGET_FOLLOW_MAX_TARGETS) ||
       (0U == g_target_follow_state.targets[index].valid) ||
       (0U != g_target_follow_state.targets[index].reached))
    {
        return 0U;
    }

    distance = target_follow_distance(g_target_follow_state.targets[index].strafe_m,
                                      g_target_follow_state.targets[index].forward_m,
                                      g_target_follow_state.tag_strafe_m,
                                      g_target_follow_state.tag_forward_m);
    if(0 != distance_m)
    {
        *distance_m = distance;
    }

    if((distance <= TARGET_FOLLOW_TARGET_MATCH_RADIUS_M) &&
       (distance <= TARGET_FOLLOW_TAG_GUARD_RADIUS_M))
    {
        return 1U;
    }

    return 0U;
}

static uint8 target_follow_find_candidate(float *candidate_distance_m)
{
    uint8 i;
    uint8 best_index;
    float best_distance;
    float distance;

    best_index = TARGET_FOLLOW_INVALID_INDEX;
    best_distance = 0.0f;

    if((g_target_follow_state.active_index != TARGET_FOLLOW_INVALID_INDEX) &&
       (0U != target_follow_target_is_eligible(g_target_follow_state.active_index, &distance)))
    {
        if(0 != candidate_distance_m)
        {
            *candidate_distance_m = distance;
        }
        return g_target_follow_state.active_index;
    }

    for(i = 0U; i < g_target_follow_state.target_count; i++)
    {
        if(0U == target_follow_target_is_eligible(i, &distance))
        {
            continue;
        }

        if((TARGET_FOLLOW_INVALID_INDEX == best_index) || (distance < best_distance))
        {
            best_index = i;
            best_distance = distance;
        }
    }

    if((TARGET_FOLLOW_INVALID_INDEX != best_index) && (0 != candidate_distance_m))
    {
        *candidate_distance_m = best_distance;
    }

    return best_index;
}

static void target_follow_update_tag_position(void)
{
    float rel_x_cm;
    float rel_y_cm;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float rel_strafe_m;
    float rel_forward_m;

    rel_x_cm = 0.0f;
    rel_y_cm = 0.0f;
    (void)ALX_AOA_GetFilteredXY(&rel_x_cm, &rel_y_cm);

    rel_strafe_m = -rel_x_cm * 0.01f;
    rel_forward_m = rel_y_cm * 0.01f;
    yaw_rad = control_get_current_yaw_angle();
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    g_target_follow_state.car_strafe_m = g_odometer.strafe_distance;
    g_target_follow_state.car_forward_m = g_odometer.forward_distance;
    g_target_follow_state.tag_relative_strafe_m = rel_strafe_m;
    g_target_follow_state.tag_relative_forward_m = rel_forward_m;
    g_target_follow_state.tag_forward_m =
        g_target_follow_state.car_forward_m +
        ((cos_yaw * rel_forward_m) - (sin_yaw * rel_strafe_m));
    g_target_follow_state.tag_strafe_m =
        g_target_follow_state.car_strafe_m +
        ((sin_yaw * rel_forward_m) + (cos_yaw * rel_strafe_m));
    g_target_follow_state.car_tag_distance_m =
        sqrtf((rel_strafe_m * rel_strafe_m) + (rel_forward_m * rel_forward_m));
    g_target_follow_state.car_in_tag_range =
        (g_target_follow_state.car_tag_distance_m <= TARGET_FOLLOW_TAG_GUARD_RADIUS_M) ? 1U : 0U;
}

static void target_follow_drive_to_target(uint8 index)
{
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float error_strafe_global;
    float error_forward_global;
    float error_strafe_body;
    float error_forward_body;

    error_strafe_global =
        g_target_follow_state.targets[index].strafe_m - g_target_follow_state.car_strafe_m;
    error_forward_global =
        g_target_follow_state.targets[index].forward_m - g_target_follow_state.car_forward_m;
    g_target_follow_state.target_error_strafe_m = error_strafe_global;
    g_target_follow_state.target_error_forward_m = error_forward_global;
    g_target_follow_state.target_car_distance_m =
        sqrtf((error_strafe_global * error_strafe_global) +
              (error_forward_global * error_forward_global));

    if(g_target_follow_state.target_car_distance_m <= TARGET_FOLLOW_REACHED_RADIUS_M)
    {
        g_target_follow_state.targets[index].reached = 1U;
        g_target_follow_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
        g_target_follow_state.mode = TARGET_FOLLOW_MODE_TARGET_REACHED;
        g_target_follow_state.forward_target = 0.0f;
        g_target_follow_state.strafe_target = 0.0f;
        g_target_follow_state.target_pid_forward_output = 0.0f;
        g_target_follow_state.target_pid_strafe_output = 0.0f;
        g_target_follow_state.output_valid = 1U;
        target_follow_pid_init();
        return;
    }

    yaw_rad = control_get_current_yaw_angle();
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);
    error_forward_body = (cos_yaw * error_forward_global) + (sin_yaw * error_strafe_global);
    error_strafe_body = (-sin_yaw * error_forward_global) + (cos_yaw * error_strafe_global);
    error_forward_body = target_follow_deadband(error_forward_body, TARGET_FOLLOW_POSITION_DEADBAND_M);
    error_strafe_body = target_follow_deadband(error_strafe_body, TARGET_FOLLOW_POSITION_DEADBAND_M);

    g_target_follow_state.forward_target =
        target_follow_limit(PositionalPID_Update(&s_target_forward_pid, error_forward_body, 0.0f),
                            TARGET_FOLLOW_OUTPUT_LIMIT);
    g_target_follow_state.strafe_target =
        target_follow_limit(PositionalPID_Update(&s_target_strafe_pid, error_strafe_body, 0.0f),
                            TARGET_FOLLOW_OUTPUT_LIMIT);
    g_target_follow_state.target_pid_forward_output = g_target_follow_state.forward_target;
    g_target_follow_state.target_pid_strafe_output = g_target_follow_state.strafe_target;
    target_follow_apply_vector_limit();
    g_target_follow_state.target_pid_forward_output = g_target_follow_state.forward_target;
    g_target_follow_state.target_pid_strafe_output = g_target_follow_state.strafe_target;
    g_target_follow_state.mode = TARGET_FOLLOW_MODE_GOTO_TARGET;
    g_target_follow_state.output_valid = 1U;
}

void target_follow_init(void)
{
    target_follow_clear_targets();
    target_follow_reset();
}

void target_follow_reset(void)
{
    target_follow_pid_init();
    target_follow_clear_runtime();
}

void target_follow_restart_targets(void)
{
    uint8 i;

    for(i = 0U; i < TARGET_FOLLOW_MAX_TARGETS; i++)
    {
        g_target_follow_state.targets[i].reached = 0U;
    }

    target_follow_reset();
}

void target_follow_clear_targets(void)
{
    uint8 i;

    for(i = 0U; i < TARGET_FOLLOW_MAX_TARGETS; i++)
    {
        g_target_follow_state.targets[i].strafe_m = 0.0f;
        g_target_follow_state.targets[i].forward_m = 0.0f;
        g_target_follow_state.targets[i].reached = 0U;
        g_target_follow_state.targets[i].valid = 0U;
    }

    g_target_follow_state.target_count = 0U;
    g_target_follow_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
    g_target_follow_state.candidate_index = TARGET_FOLLOW_INVALID_INDEX;
}

uint8 target_follow_add_target(float strafe_m, float forward_m)
{
    if(g_target_follow_state.target_count >= TARGET_FOLLOW_MAX_TARGETS)
    {
        return 0U;
    }

    return target_follow_set_target(g_target_follow_state.target_count, strafe_m, forward_m);
}

uint8 target_follow_set_target(uint8 index, float strafe_m, float forward_m)
{
    if(index >= TARGET_FOLLOW_MAX_TARGETS)
    {
        return 0U;
    }

    g_target_follow_state.targets[index].strafe_m = strafe_m;
    g_target_follow_state.targets[index].forward_m = forward_m;
    g_target_follow_state.targets[index].reached = 0U;
    g_target_follow_state.targets[index].valid = 1U;

    if(index >= g_target_follow_state.target_count)
    {
        g_target_follow_state.target_count = (uint8)(index + 1U);
    }

    return 1U;
}

void target_follow_update(uint32 now_ms)
{
    float tag_target_distance;
    float rel_x_cm;
    float rel_y_cm;
    uint8 candidate;

    tag_target_distance = 0.0f;
    rel_x_cm = 0.0f;
    rel_y_cm = 0.0f;
    g_target_follow_state.forward_target = 0.0f;
    g_target_follow_state.strafe_target = 0.0f;
    g_target_follow_state.target_pid_forward_output = 0.0f;
    g_target_follow_state.target_pid_strafe_output = 0.0f;
    g_target_follow_state.output_valid = 0U;
    g_target_follow_state.target_in_tag_range = 0U;
    g_target_follow_state.candidate_index = TARGET_FOLLOW_INVALID_INDEX;

    uwb_follow_update(now_ms);
    g_target_follow_state.tag_online = ALX_AOA_IsTagOnline(now_ms, TARGET_FOLLOW_UWB_TIMEOUT_MS);
    if((0U == g_target_follow_state.tag_online) ||
       (0U == ALX_AOA_GetFilteredXY(&rel_x_cm, &rel_y_cm)))
    {
        target_follow_pid_init();
        g_target_follow_state.mode = TARGET_FOLLOW_MODE_IDLE;
        return;
    }

    target_follow_update_tag_position();
    if(0U == g_target_follow_state.car_in_tag_range)
    {
        g_target_follow_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
        target_follow_pid_init();
        target_follow_copy_tag_follow_output();
        return;
    }

    candidate = target_follow_find_candidate(&tag_target_distance);
    g_target_follow_state.candidate_index = candidate;
    g_target_follow_state.target_tag_distance_m = tag_target_distance;

    if(TARGET_FOLLOW_INVALID_INDEX == candidate)
    {
        g_target_follow_state.active_index = TARGET_FOLLOW_INVALID_INDEX;
        target_follow_pid_init();
        target_follow_copy_tag_follow_output();
        return;
    }

    if(g_target_follow_state.active_index != candidate)
    {
        target_follow_pid_init();
    }

    g_target_follow_state.active_index = candidate;
    g_target_follow_state.target_in_tag_range = 1U;
    target_follow_drive_to_target(candidate);
}
