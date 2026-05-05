#include "zf_common_headfile.h"
#include "motor.h"



/**
 * @brief  初始化四轮电机
 */
void mecanum_motor_init(void)
{
    // 电机1 - 左前
    gpio_init(MOTOR_M1_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M1_PWM, 17000, 0);

    // 电机2 - 右前
    gpio_init(MOTOR_M2_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M2_PWM, 17000, 0);

    // 电机3 - 左后
    gpio_init(MOTOR_M3_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M3_PWM, 17000, 0);

    // 电机4 - 右后
    gpio_init(MOTOR_M4_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    pwm_init(MOTOR_M4_PWM, 17000, 0);
}

/*
------------------------------------------------------------------------------------------------------------------
函数简介     pwm限幅
参数说明     
返回参数     
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
int16_t speed_limit(int16_t speed)
{
    if(speed > MOTOR_PWM_MAX)
    {
        speed = MOTOR_PWM_MAX;
    }
    else if(speed < (-MOTOR_PWM_MAX))
    {
        speed = -MOTOR_PWM_MAX;
    }
    return speed;
}

/**
 * @brief  设置单个电机速度
 * @param  dir_pin: 方向控制引脚
 * @param  pwm_ch: PWM通道
 * @param  speed: 速度值（正值正转，负值反转）
 * @param  invert: 是否反转方向 (1=反转, 0=不反转)
 */
void motor_set_single(gpio_pin_enum dir_pin, pwm_channel_enum pwm_ch, int16_t speed, uint8_t invert)
{
    //speed = speed_limit(speed);

    // 如果需要反转，取反速度
    if (invert)
    {
        speed = -speed;
    }

    if (speed >= 0)
    {
        gpio_set_level(dir_pin, GPIO_LOW);      // 正转
        pwm_set_duty(pwm_ch, speed);
    }
    else
    {
        gpio_set_level(dir_pin, GPIO_HIGH);     // 反转
        pwm_set_duty(pwm_ch, -speed);
    }
}

/**
 * @brief  设置电机1（左前）速度
 */
void motor_m1_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M1_DIR, MOTOR_M1_PWM, speed, MOTOR_M1_INVERT);
}

/**
 * @brief  设置电机2（右前）速度
 */
void motor_m2_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M2_DIR, MOTOR_M2_PWM, speed, MOTOR_M2_INVERT);
}

/**
 * @brief  设置电机3（左后）速度
 */
void motor_m3_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M3_DIR, MOTOR_M3_PWM, speed, MOTOR_M3_INVERT);
}

/**
 * @brief  设置电机4（右后）速度
 */
void motor_m4_set_speed(int16_t speed)
{
    motor_set_single(MOTOR_M4_DIR, MOTOR_M4_PWM, speed, MOTOR_M4_INVERT);
}

/**
 * @brief  同时设置四轮电机速度
 */
void mecanum_motor_set_all(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    motor_m1_set_speed(m1);
    motor_m2_set_speed(m2);
    motor_m3_set_speed(m3);
    motor_m4_set_speed(m4);
}

/**
 * @brief  停止所有电机
 */
void mecanum_motor_stop(void)
{
    mecanum_motor_set_all(0, 0, 0, 0);
}
