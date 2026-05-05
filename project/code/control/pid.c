#include "zf_common_headfile.h"
#include "pid.h"


void PositionalPID_Init(PositionalPID* pid,float kp_2,float kp_1,float ki,float kd,float i_limit,float output_limit) 
{
	pid->kp_2 = kp_2;
    pid->kp_1 = kp_1;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
    pid->i_limit = i_limit;
    pid->output_limit = output_limit;
}

float PositionalPID_Update(PositionalPID* pid, float target, float current)
{
	float err = target - current;

	pid->integral += err;
			
	if(pid->integral > pid->i_limit)pid->integral = pid->i_limit;
	else if(pid->integral < -pid->i_limit)pid->integral = -pid->i_limit;
		
	float derivative = err - pid->prev_err;
			
	float output = 0;

    float d = pid->kd * derivative;
		
	if(err >= 0)
	{
			 output = pid->kp_2 * err * err + pid->kp_1 * err
					+ pid->ki * pid->integral 
					+ d;
	}
	else
	{
			output = - pid->kp_2 * err * err + pid->kp_1 * err
        			+ pid->ki * pid->integral 
					+ d;		
    }

    pid->prev_err = err;
							
	if(output > pid->output_limit)output = pid->output_limit;
	if(output < -pid->output_limit)output = -pid->output_limit;			
	return output;
}

void IncrementPID_Init (IncrementPID* pid,float kp,float ki,float kd,float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->output_limit = output_limit;
}

float IncreamPID_Update(IncrementPID *pid, float target, float current) 
{
    float error = target - current;
    
    float p_term = pid->kp * (error - pid->last_error);
    float i_term = pid->ki * error;       
    float d_term = pid->kd * (error - 2*pid->last_error + pid->prev_error);
    float increment = p_term + i_term + d_term;
    pid->output += increment;

    pid->prev_error = pid->last_error;
    pid->last_error = error;

    if(pid->output > pid->output_limit) pid->output = pid->output_limit;
    if(pid->output < -pid->output_limit) pid->output = -pid->output_limit;
    
    return pid->output;
}
	
