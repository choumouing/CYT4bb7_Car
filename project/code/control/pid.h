/*********************************************************************************************************************
* PID控制器模块 - 头文件
*
* 位置式PID：用于航向角控制（外环）
*   输入/输出单位：角度(°) → 角速度(rad/s)
*
* 增量式PID：用于轮速控制（内环）
*   输入/输出单位：编码器计数(pulse) → PWM增量
********************************************************************************************************************/

#ifndef __PID_H__
#define __PID_H__

typedef struct
{
    float kp_2, kp_1, ki, kd;
    float integral;
    float prev_err;
    float i_limit;
    float output_limit;
} PositionalPID;

typedef struct {
    float kp, ki, kd;
    float last_error;
    float prev_error;
    float output;
    float output_limit;
} IncrementPID;

void PositionalPID_Init(PositionalPID* pid, float kp_2, float kp_1, float ki, float kd,
                        float i_limit, float output_limit);
float PositionalPID_Update(PositionalPID* pid, float target, float current);

void IncrementPID_Init(IncrementPID* pid, float kp, float ki, float kd,
                       float output_limit);
float IncreamPID_Update(IncrementPID *pid, float target, float current);

#endif
