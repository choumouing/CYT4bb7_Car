#include "velocity_fusion.h"

#define VELOCITY_FUSION_DT_S                    (0.01f)
#define VELOCITY_FUSION_FORWARD_COUNT_PER_M     (11287.0f)
#define VELOCITY_FUSION_RIGHT_COUNT_PER_M       (12100.0f)

#define VELOCITY_FUSION_ALPHA                   (0.10f)
#define VELOCITY_FUSION_BETA                    (0.018f)
#define VELOCITY_FUSION_VEL_GAIN                (0.95f)
#define VELOCITY_FUSION_VEL_LIMIT_CMPS          (250.0f)
#define VELOCITY_FUSION_VEL_INNOV_LIMIT_CMPS    (80.0f)
#define VELOCITY_FUSION_POS_RES_LIMIT_CM        (30.0f)
#define VELOCITY_FUSION_POS_GATE_CM             (80.0f)
#define VELOCITY_FUSION_UWB_DT_DEFAULT_S        (0.04f)
#define VELOCITY_FUSION_UWB_DT_MIN_S            (0.02f)
#define VELOCITY_FUSION_UWB_DT_MAX_S            (0.20f)
#define VELOCITY_FUSION_POS_LEAD_TIME_S         (0.18f)
#define VELOCITY_FUSION_POS_LEAD_LIMIT_CM       (12.0f)

#define VELOCITY_FUSION_DEG_TO_RAD              (0.017453292519943295f)

static velocity_fusion_state_t s_velocity_fusion_state;
static float s_pos_world_right_cm = 0.0f;
static float s_pos_world_forward_cm = 0.0f;
static float s_vel_world_right_cmps = 0.0f;
static float s_vel_world_forward_cmps = 0.0f;
static float s_residual_world_right_cm = 0.0f;
static float s_residual_world_forward_cm = 0.0f;
static uint32 s_last_uwb_position_ms = 0U;

static float velocity_fusion_limit_abs(float value, float limit)
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

static void velocity_fusion_rotate_yaw(float right_in,
                                       float forward_in,
                                       float yaw_deg,
                                       float *right_out,
                                       float *forward_out)
{
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;

    yaw_rad = yaw_deg * VELOCITY_FUSION_DEG_TO_RAD;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    *right_out = (cos_yaw * right_in) + (sin_yaw * forward_in);
    *forward_out = (-sin_yaw * right_in) + (cos_yaw * forward_in);
}

static void velocity_fusion_apply_position_lead(float vel_right_cmps,
                                                float vel_forward_cmps,
                                                float *pos_right_cm,
                                                float *pos_forward_cm)
{
    float lead_right_cm;
    float lead_forward_cm;
    float lead_norm_cm;
    float scale;

    lead_right_cm = vel_right_cmps * VELOCITY_FUSION_POS_LEAD_TIME_S;
    lead_forward_cm = vel_forward_cmps * VELOCITY_FUSION_POS_LEAD_TIME_S;
    lead_norm_cm = sqrtf((lead_right_cm * lead_right_cm) +
                         (lead_forward_cm * lead_forward_cm));

    if(lead_norm_cm > VELOCITY_FUSION_POS_LEAD_LIMIT_CM)
    {
        scale = VELOCITY_FUSION_POS_LEAD_LIMIT_CM / lead_norm_cm;
        lead_right_cm *= scale;
        lead_forward_cm *= scale;
    }

    *pos_right_cm += lead_right_cm;
    *pos_forward_cm += lead_forward_cm;
}

static uint8 velocity_fusion_is_air_valid(void)
{
    if(0U == air_comm_car_is_online())
    {
        return 0U;
    }

    return 1U;
}

static void velocity_fusion_publish_body_state(void)
{
    float pos_right_cm;
    float pos_forward_cm;
    float vel_right_cmps;
    float vel_forward_cmps;

    velocity_fusion_rotate_yaw(s_pos_world_right_cm,
                               s_pos_world_forward_cm,
                               -g_euler.yaw,
                               &pos_right_cm,
                               &pos_forward_cm);
    velocity_fusion_rotate_yaw(s_vel_world_right_cmps,
                               s_vel_world_forward_cmps,
                               -g_euler.yaw,
                               &vel_right_cmps,
                               &vel_forward_cmps);
    velocity_fusion_apply_position_lead(vel_right_cmps,
                                        vel_forward_cmps,
                                        &pos_right_cm,
                                        &pos_forward_cm);
    s_velocity_fusion_state.pos_right_cm = pos_right_cm;
    s_velocity_fusion_state.pos_forward_cm = pos_forward_cm;
    s_velocity_fusion_state.vel_right_cmps = vel_right_cmps;
    s_velocity_fusion_state.vel_forward_cmps = vel_forward_cmps;
    velocity_fusion_rotate_yaw(s_residual_world_right_cm,
                               s_residual_world_forward_cm,
                               -g_euler.yaw,
                               &s_velocity_fusion_state.residual_right_cm,
                               &s_velocity_fusion_state.residual_forward_cm);
    s_velocity_fusion_state.valid = 1U;
}

static uint8 velocity_fusion_read_measurement(uint32 now_ms,
                                             float *pos_right_cm,
                                             float *pos_forward_cm,
                                             float *rel_vel_right_cmps,
                                             float *rel_vel_forward_cmps,
                                             uint32 *uwb_position_ms)
{
    ALX_AOA_Position_t position;
    float uwb_x_cm;
    float uwb_y_cm;
    float left_front;
    float right_front;
    float left_rear;
    float right_rear;
    float car_forward_count;
    float car_right_count;
    float car_right_body_cmps;
    float car_forward_body_cmps;
    float air_right_body_cmps;
    float air_forward_body_cmps;
    float air_right_world_cmps;
    float air_forward_world_cmps;
    float car_right_world_cmps;
    float car_forward_world_cmps;
    float pos_right_world_cm;
    float pos_forward_world_cm;

    if(0U == velocity_fusion_is_air_valid())
    {
        return 0U;
    }

    if((0U == ALX_AOA_IsTagOnline(now_ms, UWB_FOLLOW_TIMEOUT_MS)) ||
       (0U == ALX_AOA_GetLatest(&position)) ||
       (0U == ALX_AOA_GetFilteredXY(&uwb_x_cm, &uwb_y_cm)))
    {
        return 0U;
    }

    air_right_body_cmps = -g_air_vel_x;
    air_forward_body_cmps = g_air_vel_y;
    velocity_fusion_rotate_yaw(air_right_body_cmps,
                               air_forward_body_cmps,
                               g_air_yaw_deg,
                               &air_right_world_cmps,
                               &air_forward_world_cmps);

    left_front = encoder_get_left_front_filtered_count();
    right_front = encoder_get_right_front_filtered_count();
    left_rear = encoder_get_left_rear_filtered_count();
    right_rear = encoder_get_right_rear_filtered_count();

    car_forward_count = (left_front + right_front + left_rear + right_rear) * 0.25f;
    car_right_count = (left_front - right_front - left_rear + right_rear) * 0.25f;
    car_forward_body_cmps = car_forward_count * 10000.0f / VELOCITY_FUSION_FORWARD_COUNT_PER_M;
    car_right_body_cmps = car_right_count * 10000.0f / VELOCITY_FUSION_RIGHT_COUNT_PER_M;
    velocity_fusion_rotate_yaw(car_right_body_cmps,
                               car_forward_body_cmps,
                               g_euler.yaw,
                               &car_right_world_cmps,
                               &car_forward_world_cmps);

    velocity_fusion_rotate_yaw(uwb_x_cm,
                               uwb_y_cm,
                               g_euler.yaw,
                               &pos_right_world_cm,
                               &pos_forward_world_cm);

    *pos_right_cm = pos_right_world_cm;
    *pos_forward_cm = pos_forward_world_cm;
    *rel_vel_right_cmps = air_right_world_cmps - car_right_world_cmps;
    *rel_vel_forward_cmps = air_forward_world_cmps - car_forward_world_cmps;
    *uwb_position_ms = position.last_position_ms;

    s_velocity_fusion_state.air_right_cmps = air_right_world_cmps;
    s_velocity_fusion_state.air_forward_cmps = air_forward_world_cmps;
    s_velocity_fusion_state.car_right_cmps = car_right_world_cmps;
    s_velocity_fusion_state.car_forward_cmps = car_forward_world_cmps;
    s_velocity_fusion_state.uwb_right_cm = uwb_x_cm;
    s_velocity_fusion_state.uwb_forward_cm = uwb_y_cm;

    return 1U;
}

void velocity_fusion_init(void)
{
    velocity_fusion_reset();
}

void velocity_fusion_reset(void)
{
    memset(&s_velocity_fusion_state, 0, sizeof(s_velocity_fusion_state));
    s_pos_world_right_cm = 0.0f;
    s_pos_world_forward_cm = 0.0f;
    s_vel_world_right_cmps = 0.0f;
    s_vel_world_forward_cmps = 0.0f;
    s_residual_world_right_cm = 0.0f;
    s_residual_world_forward_cm = 0.0f;
    s_last_uwb_position_ms = 0U;
}

void velocity_fusion_update_100HZ(uint32 now_ms)
{
    float pos_meas_right_cm;
    float pos_meas_forward_cm;
    float vel_meas_right_cmps;
    float vel_meas_forward_cmps;
    float vel_error_right_cmps;
    float vel_error_forward_cmps;
    float residual_right_cm;
    float residual_forward_cm;
    float residual_norm_cm;
    float dt_uwb_s;
    uint32 uwb_position_ms;
    uint8 uwb_updated;

    s_velocity_fusion_state.uwb_updated = 0U;

    if(0U == velocity_fusion_read_measurement(now_ms,
                                             &pos_meas_right_cm,
                                             &pos_meas_forward_cm,
                                             &vel_meas_right_cmps,
                                             &vel_meas_forward_cmps,
                                             &uwb_position_ms))
    {
        velocity_fusion_reset();
        return;
    }

    if(0U == s_velocity_fusion_state.valid)
    {
        s_pos_world_right_cm = pos_meas_right_cm;
        s_pos_world_forward_cm = pos_meas_forward_cm;
        s_vel_world_right_cmps =
            velocity_fusion_limit_abs(vel_meas_right_cmps, VELOCITY_FUSION_VEL_LIMIT_CMPS);
        s_vel_world_forward_cmps =
            velocity_fusion_limit_abs(vel_meas_forward_cmps, VELOCITY_FUSION_VEL_LIMIT_CMPS);
        s_residual_world_right_cm = 0.0f;
        s_residual_world_forward_cm = 0.0f;
        s_velocity_fusion_state.uwb_updated = 1U;
        velocity_fusion_publish_body_state();
        s_last_uwb_position_ms = uwb_position_ms;
        return;
    }

    vel_error_right_cmps = velocity_fusion_limit_abs(vel_meas_right_cmps -
                                                    s_vel_world_right_cmps,
                                                    VELOCITY_FUSION_VEL_INNOV_LIMIT_CMPS);
    vel_error_forward_cmps = velocity_fusion_limit_abs(vel_meas_forward_cmps -
                                                      s_vel_world_forward_cmps,
                                                      VELOCITY_FUSION_VEL_INNOV_LIMIT_CMPS);

    s_vel_world_right_cmps += VELOCITY_FUSION_VEL_GAIN * vel_error_right_cmps;
    s_vel_world_forward_cmps += VELOCITY_FUSION_VEL_GAIN * vel_error_forward_cmps;
    s_vel_world_right_cmps =
        velocity_fusion_limit_abs(s_vel_world_right_cmps,
                                  VELOCITY_FUSION_VEL_LIMIT_CMPS);
    s_vel_world_forward_cmps =
        velocity_fusion_limit_abs(s_vel_world_forward_cmps,
                                  VELOCITY_FUSION_VEL_LIMIT_CMPS);

    s_pos_world_right_cm += s_vel_world_right_cmps * VELOCITY_FUSION_DT_S;
    s_pos_world_forward_cm += s_vel_world_forward_cmps * VELOCITY_FUSION_DT_S;

    uwb_updated = (uwb_position_ms != s_last_uwb_position_ms) ? 1U : 0U;
    if(0U == uwb_updated)
    {
        velocity_fusion_publish_body_state();
        return;
    }

    if((0U != s_last_uwb_position_ms) && (uwb_position_ms > s_last_uwb_position_ms))
    {
        dt_uwb_s = (float)(uwb_position_ms - s_last_uwb_position_ms) * 0.001f;
        dt_uwb_s = car_math_clampf(dt_uwb_s,
                                   VELOCITY_FUSION_UWB_DT_MIN_S,
                                   VELOCITY_FUSION_UWB_DT_MAX_S);
    }
    else
    {
        dt_uwb_s = VELOCITY_FUSION_UWB_DT_DEFAULT_S;
    }

    residual_right_cm = pos_meas_right_cm - s_pos_world_right_cm;
    residual_forward_cm = pos_meas_forward_cm - s_pos_world_forward_cm;
    residual_norm_cm = sqrtf((residual_right_cm * residual_right_cm) +
                             (residual_forward_cm * residual_forward_cm));
    if(residual_norm_cm > VELOCITY_FUSION_POS_GATE_CM)
    {
        s_pos_world_right_cm = pos_meas_right_cm;
        s_pos_world_forward_cm = pos_meas_forward_cm;
        s_vel_world_right_cmps =
            velocity_fusion_limit_abs(vel_meas_right_cmps, VELOCITY_FUSION_VEL_LIMIT_CMPS);
        s_vel_world_forward_cmps =
            velocity_fusion_limit_abs(vel_meas_forward_cmps, VELOCITY_FUSION_VEL_LIMIT_CMPS);
        s_residual_world_right_cm = 0.0f;
        s_residual_world_forward_cm = 0.0f;
    }
    else
    {
        residual_right_cm = velocity_fusion_limit_abs(residual_right_cm,
                                                      VELOCITY_FUSION_POS_RES_LIMIT_CM);
        residual_forward_cm = velocity_fusion_limit_abs(residual_forward_cm,
                                                        VELOCITY_FUSION_POS_RES_LIMIT_CM);

        s_pos_world_right_cm += VELOCITY_FUSION_ALPHA * residual_right_cm;
        s_pos_world_forward_cm += VELOCITY_FUSION_ALPHA * residual_forward_cm;
        s_vel_world_right_cmps += (VELOCITY_FUSION_BETA * residual_right_cm) / dt_uwb_s;
        s_vel_world_forward_cmps += (VELOCITY_FUSION_BETA * residual_forward_cm) / dt_uwb_s;
        s_vel_world_right_cmps =
            velocity_fusion_limit_abs(s_vel_world_right_cmps,
                                      VELOCITY_FUSION_VEL_LIMIT_CMPS);
        s_vel_world_forward_cmps =
            velocity_fusion_limit_abs(s_vel_world_forward_cmps,
                                      VELOCITY_FUSION_VEL_LIMIT_CMPS);
        s_residual_world_right_cm = residual_right_cm;
        s_residual_world_forward_cm = residual_forward_cm;
    }

    s_last_uwb_position_ms = uwb_position_ms;
    s_velocity_fusion_state.uwb_updated = 1U;
    velocity_fusion_publish_body_state();
}

uint8 velocity_fusion_get_state(velocity_fusion_state_t *state)
{
    if(0 == state)
    {
        return 0U;
    }

    if(0U != s_velocity_fusion_state.valid)
    {
        velocity_fusion_publish_body_state();
    }

    *state = s_velocity_fusion_state;
    return s_velocity_fusion_state.valid;
}
