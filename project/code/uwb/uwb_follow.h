#ifndef UWB_FOLLOW_H
#define UWB_FOLLOW_H

#include "zf_common_headfile.h"
#include "navigation/odometry.h"
#include "control/car_control.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_FOLLOW_BASE_PERIOD_MS       (5U)
#define UWB_FOLLOW_CONTROL_PERIOD_MS    (20U)
#define UWB_FOLLOW_TIMEOUT_MS           (150U)

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float prev_d;
    float integral_limit;
    float output_limit;
    float d_lpf;
} uwb_follow_pid_t;

typedef struct {
    odometry_t *odometry;
    cascade_controller_t *controller;
    uwb_follow_pid_t x_pid;
    uwb_follow_pid_t y_pid;
    uint8 active;
    uint8 tag_online;
    uint8 control_divider;
    float hold_yaw_deg;
    float tag_rel_x_mm;
    float tag_rel_y_mm;
    float target_x_mm;
    float target_y_mm;
    float error_x_mm;
    float error_y_mm;
    float vx_body_target;
    float vy_body_target;
} uwb_follow_task_t;

void uwb_follow_init(uwb_follow_task_t *task,
                     odometry_t *odometry,
                     cascade_controller_t *controller);

void uwb_follow_start(uwb_follow_task_t *task, float current_yaw_deg);

void uwb_follow_stop(uwb_follow_task_t *task);

void uwb_follow_control_loop_5ms(uwb_follow_task_t *task, uint32 now_ms);

#ifdef __cplusplus
}
#endif

#endif
