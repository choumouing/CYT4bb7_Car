/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#include "pid.h"


/* 初始化位置式PID，清零所有状态 */
void PositionalPID_Init(PositionalPID* pid,float kp_2,float kp_1,float ki,float kd,float i_limit,float output_limit)
{
    pid->kp_2 = kp_2;
    pid->kp_1 = kp_1;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->output = 0.0f;
    pid->i_limit = i_limit;
    pid->output_limit = output_limit;
}

/* 位置式PID计算
 * 抗饱和策略：积分值钳位在±i_limit
 * p项特殊：正误差用kp_2*err^2，负误差用-kp_2*err^2，确保二次项平滑过渡
 * 输出最终钳位在±output_limit
 */
float PositionalPID_Update(PositionalPID* pid, float target, float current)
{
    float err = target - current;

    pid->integral += err;

    /* 积分限幅（抗饱和） */
    if(pid->integral > pid->i_limit)pid->integral = pid->i_limit;
    else if(pid->integral < -pid->i_limit)pid->integral = -pid->i_limit;

    float derivative = err - pid->prev_err;

    float output = 0;
    float p = 0.0f;
    float i = pid->ki * pid->integral;
    float d = pid->kd * derivative;

    /* 二次+线性p项，误差正负时二次项系数符号相反 */
    if(err >= 0)
    {
             p = pid->kp_2 * err * err + pid->kp_1 * err;
    }
    else
    {
            p = - pid->kp_2 * err * err + pid->kp_1 * err;
    }

    output = p + i + d;
    pid->p_term = p;
    pid->i_term = i;
    pid->d_term = d;
    pid->prev_err = err;

    /* 输出限幅 */
    if(output > pid->output_limit)output = pid->output_limit;
    if(output < -pid->output_limit)output = -pid->output_limit;
    pid->output = output;
    return output;
}

/* 初始化增量式PID，清零所有状态 */
void IncrementPID_Init (IncrementPID* pid,float kp,float ki,float kd,float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->increment = 0.0f;
    pid->output = 0.0f;
    pid->output_limit = output_limit;
}

/* 增量式PID计算
 * 增量公式：Δu = kp*(e(k)-e(k-1)) + ki*e(k) + kd*(e(k)-2e(k-1)+e(k-2))
 * output += increment（累加型）
 * 输出最终钳位在±output_limit
 * 注意：没有积分限幅，因为增量式天然抗饱和
 */
float IncrementPID_Update(IncrementPID *pid, float target, float current)
{
    float error = target - current;

    float p_term = pid->kp * (error - pid->last_error);
    float i_term = pid->ki * error;
    float d_term = pid->kd * (error - 2*pid->last_error + pid->prev_error);
    float increment = p_term + i_term + d_term;
    pid->output += increment;
    pid->p_term = p_term;
    pid->i_term = i_term;
    pid->d_term = d_term;
    pid->increment = increment;

    pid->prev_error = pid->last_error;
    pid->last_error = error;

    /* 输出限幅 */
    if(pid->output > pid->output_limit) pid->output = pid->output_limit;
    if(pid->output < -pid->output_limit) pid->output = -pid->output_limit;

    return pid->output;
}
