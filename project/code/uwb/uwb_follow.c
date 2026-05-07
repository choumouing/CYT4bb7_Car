#include "uwb_follow.h"

#include "uwb/ALX_AOA.h"
#include "Attitude/IMU_TOP.h"
#include "encoder/encoder_control.h"
#include "menu/menu_config.h"
#include "motor/motor.h"
#include <math.h>

#define UWB_FOLLOW_DEG_TO_RAD              (0.017453292519943295f)
#define UWB_FOLLOW_GYRO_DPS_TO_RAD_S       (0.017453292519943295f)
#define UWB_FOLLOW_CM_TO_MM                (10.0f)
#define UWB_FOLLOW_DT_S                    (((float)UWB_FOLLOW_CONTROL_PERIOD_MS) * 0.001f)
#define UWB_FOLLOW_CONTROL_DIVIDER         (UWB_FOLLOW_CONTROL_PERIOD_MS / UWB_FOLLOW_BASE_PERIOD_MS)
#define UWB_FOLLOW_POS_DEADBAND_MM         (80.0f)

static float uwb_follow_clamp(float value, float min_value, float max_value)
{
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}

static void uwb_follow_apply_pid_params(uwb_follow_task_t *task)
{
    task->x_pid.kp = uwb_x_kp;
    task->x_pid.ki = uwb_x_ki;
    task->x_pid.kd = uwb_x_kd;
    task->x_pid.integral_limit = uwb_x_i_limit;
    task->x_pid.output_limit = uwb_x_output_limit;
    task->x_pid.d_lpf = uwb_x_d_lpf;

    task->y_pid.kp = uwb_y_kp;
    task->y_pid.ki = uwb_y_ki;
    task->y_pid.kd = uwb_y_kd;
    task->y_pid.integral_limit = uwb_y_i_limit;
    task->y_pid.output_limit = uwb_y_output_limit;
    task->y_pid.d_lpf = uwb_y_d_lpf;
}

static void uwb_follow_pid_reset(uwb_follow_pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_d = 0.0f;
}

static float uwb_follow_pid_update(uwb_follow_pid_t *pid, float error, float dt_s)
{
    float derivative;
    float d_filtered;
    float output;

    pid->integral += error * dt_s;
    pid->integral = uwb_follow_clamp(pid->integral,
                                     -pid->integral_limit,
                                     pid->integral_limit);

    derivative = (error - pid->prev_error) / dt_s;
    d_filtered = (1.0f - pid->d_lpf) * derivative + pid->d_lpf * pid->prev_d;

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * d_filtered;
    output = uwb_follow_clamp(output, -pid->output_limit, pid->output_limit);

    pid->prev_error = error;
    pid->prev_d = d_filtered;

    return output;
}

static void uwb_follow_reset_runtime(uwb_follow_task_t *task)
{
    uwb_follow_pid_reset(&task->x_pid);
    uwb_follow_pid_reset(&task->y_pid);
    task->tag_online = 0U;
    task->control_divider = 0U;
    task->tag_rel_x_mm = 0.0f;
    task->tag_rel_y_mm = 0.0f;
    task->target_x_mm = 0.0f;
    task->target_y_mm = 0.0f;
    task->error_x_mm = 0.0f;
    task->error_y_mm = 0.0f;
    task->vx_body_target = 0.0f;
    task->vy_body_target = 0.0f;
}

static void uwb_follow_limit_vector(float *vx, float *vy, float max_value)
{
    float mag = sqrtf((*vx) * (*vx) + (*vy) * (*vy));

    if(mag > max_value && mag > 1.0e-6f) {
        float scale = max_value / mag;
        *vx *= scale;
        *vy *= scale;
    }
}

static float uwb_follow_get_speed_limit(const uwb_follow_task_t *task)
{
    float limit = task->x_pid.output_limit;

    if(task->y_pid.output_limit > limit) {
        limit = task->y_pid.output_limit;
    }

    return limit;
}

static void uwb_follow_clear_velocity(uwb_follow_task_t *task)
{
    task->vx_body_target = 0.0f;
    task->vy_body_target = 0.0f;

    if(task->controller != 0) {
        task->controller->vx_target = 0.0f;
        task->controller->vy_target = 0.0f;
    }
}

static void uwb_follow_apply_body_velocity(uwb_follow_task_t *task)
{
    if(task->controller == 0) {
        return;
    }

    task->controller->vx_target = task->vx_body_target;
    task->controller->vy_target = task->vy_body_target;
}

static void uwb_follow_update_position_loop(uwb_follow_task_t *task, uint32 now_ms)
{
    ALX_AOA_Position_t latest;
    float yaw_rad;
    float cos_yaw;
    float sin_yaw;
    float vx_body;
    float vy_body;

    task->tag_online = ALX_AOA_IsTagOnline(now_ms, UWB_FOLLOW_TIMEOUT_MS);
    uwb_follow_apply_pid_params(task);

    if(!task->tag_online || !ALX_AOA_GetLatest(&latest) || !latest.valid) {
        uwb_follow_pid_reset(&task->x_pid);
        uwb_follow_pid_reset(&task->y_pid);
        task->tag_rel_x_mm = 0.0f;
        task->tag_rel_y_mm = 0.0f;
        task->target_x_mm = task->odometry->pos_x;
        task->target_y_mm = task->odometry->pos_y;
        task->error_x_mm = 0.0f;
        task->error_y_mm = 0.0f;
        uwb_follow_clear_velocity(task);
        return;
    }

    /* UWB X is right-positive; UWB Y is forward-positive. Chassis vy is left-positive. */
    task->tag_rel_x_mm = alx_aoa_x_cm * UWB_FOLLOW_CM_TO_MM;
    task->tag_rel_y_mm = alx_aoa_y_cm * UWB_FOLLOW_CM_TO_MM;

    yaw_rad = task->odometry->pos_theta * UWB_FOLLOW_DEG_TO_RAD;
    cos_yaw = cosf(yaw_rad);
    sin_yaw = sinf(yaw_rad);

    task->target_x_mm = task->odometry->pos_x +
                        task->tag_rel_y_mm * cos_yaw +
                        task->tag_rel_x_mm * sin_yaw;
    task->target_y_mm = task->odometry->pos_y +
                        task->tag_rel_y_mm * sin_yaw -
                        task->tag_rel_x_mm * cos_yaw;

    task->error_x_mm = task->tag_rel_x_mm;
    task->error_y_mm = task->tag_rel_y_mm;

    if(fabsf(task->error_x_mm) < UWB_FOLLOW_POS_DEADBAND_MM) {
        task->error_x_mm = 0.0f;
    }
    if(fabsf(task->error_y_mm) < UWB_FOLLOW_POS_DEADBAND_MM) {
        task->error_y_mm = 0.0f;
    }

    if((0.0f == task->error_x_mm) && (0.0f == task->error_y_mm)) {
        uwb_follow_pid_reset(&task->x_pid);
        uwb_follow_pid_reset(&task->y_pid);
        uwb_follow_clear_velocity(task);
        return;
    }

    vx_body = uwb_follow_pid_update(&task->y_pid, task->error_y_mm, UWB_FOLLOW_DT_S);
    vy_body = -uwb_follow_pid_update(&task->x_pid, task->error_x_mm, UWB_FOLLOW_DT_S);

    uwb_follow_limit_vector(&vx_body, &vy_body, uwb_follow_get_speed_limit(task));

    task->vx_body_target = vx_body;
    task->vy_body_target = vy_body;
}

void uwb_follow_init(uwb_follow_task_t *task,
                     odometry_t *odometry,
                     cascade_controller_t *controller)
{
    if(task == 0) {
        return;
    }

    task->odometry = odometry;
    task->controller = controller;
    task->active = 0U;
    task->hold_yaw_deg = 0.0f;

    uwb_follow_apply_pid_params(task);
    uwb_follow_reset_runtime(task);
    ALX_AOA_Init();
}

void uwb_follow_start(uwb_follow_task_t *task, float current_yaw_deg)
{
    if(task == 0 || task->odometry == 0 || task->controller == 0) {
        return;
    }

    odometry_reset_pose(task->odometry, current_yaw_deg);
    uwb_follow_reset_runtime(task);
    task->hold_yaw_deg = task->odometry->pos_theta;
    task->active = 1U;
    is_car_running = 1.0f;
    cascade_set_target(task->controller, task->hold_yaw_deg);
}

void uwb_follow_stop(uwb_follow_task_t *task)
{
    if(task == 0) {
        return;
    }

    task->active = 0U;
    is_car_running = 0.0f;
    uwb_follow_reset_runtime(task);
    uwb_follow_clear_velocity(task);

    if(task->controller != 0) {
        cascade_controller_reset(task->controller);
    } else {
        mecanum_motor_stop();
    }
}

void uwb_follow_control_loop_5ms(uwb_follow_task_t *task, uint32 now_ms)
{
    float yaw_navigation;
    float yaw_rate_navigation;
    int16_t pwm[4];

    if(task == 0 || task->odometry == 0 || task->controller == 0) {
        return;
    }

    ALX_AOA_Update(now_ms);

    yaw_navigation = -g_euler.yaw;
    yaw_rate_navigation = -g_imufilter_1000hz.gyroz * UWB_FOLLOW_GYRO_DPS_TO_RAD_S;

    encoder_update();
    odometry_update(task->odometry, yaw_navigation);

    if((0U == task->active) || (is_car_running < 0.5f)) {
        uwb_follow_clear_velocity(task);
        cascade_controller_reset(task->controller);
        return;
    }

    task->control_divider++;
    if(task->control_divider >= UWB_FOLLOW_CONTROL_DIVIDER) {
        task->control_divider = 0U;
        uwb_follow_update_position_loop(task, now_ms);
    }

    cascade_set_target(task->controller, task->hold_yaw_deg);
    uwb_follow_apply_body_velocity(task);
    cascade_controller_update(task->controller,
                              task->odometry->pos_theta,
                              yaw_rate_navigation,
                              pwm);
}
