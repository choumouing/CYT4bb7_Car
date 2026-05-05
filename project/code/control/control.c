#include "control.h"
#include "pid.h"
#include "../encoder/encoder_control.h"
#include "../menu/menu_config.h"
#include "../motor/motor.h"

IncrementPID wheel_left_front_pid;
IncrementPID wheel_right_front_pid;
IncrementPID wheel_left_rear_pid;
IncrementPID wheel_right_rear_pid;

static void control_speed_pid_init(IncrementPID *pid)
{
    IncrementPID_Init(pid, wheel_kp, wheel_ki, wheel_kd, wheel_output_limit);
}

static void control_speed_pid_apply_params(IncrementPID *pid)
{
    pid->kp = wheel_kp;
    pid->ki = wheel_ki;
    pid->kd = wheel_kd;
    pid->output_limit = wheel_output_limit;
}

void control_speed_loop_init(void)
{
    control_speed_loop_reset();
}

void control_speed_loop_reset(void)
{
    control_speed_pid_init(&wheel_left_front_pid);
    control_speed_pid_init(&wheel_right_front_pid);
    control_speed_pid_init(&wheel_left_rear_pid);
    control_speed_pid_init(&wheel_right_rear_pid);
}

void control_speed_loop_update(float left_front_target, float right_front_target,
                               float left_rear_target, float right_rear_target)
{
    control_speed_pid_apply_params(&wheel_left_front_pid);
    control_speed_pid_apply_params(&wheel_right_front_pid);
    control_speed_pid_apply_params(&wheel_left_rear_pid);
    control_speed_pid_apply_params(&wheel_right_rear_pid);

    float left_front_pwm = IncreamPID_Update(&wheel_left_front_pid, left_front_target,
                                             encoder_get_left_front_filtered_count());
    float right_front_pwm = IncreamPID_Update(&wheel_right_front_pid, right_front_target,
                                              encoder_get_right_front_filtered_count());
    float left_rear_pwm = IncreamPID_Update(&wheel_left_rear_pid, left_rear_target,
                                            encoder_get_left_rear_filtered_count());
    float right_rear_pwm = IncreamPID_Update(&wheel_right_rear_pid, right_rear_target,
                                             encoder_get_right_rear_filtered_count());

    mecanum_motor_set_all((int16_t)left_front_pwm,
                          (int16_t)right_front_pwm,
                          (int16_t)left_rear_pwm,
                          (int16_t)right_rear_pwm);
}

void control_speed_loop_stop(void)
{
    control_speed_loop_reset();
    mecanum_motor_stop();
}
