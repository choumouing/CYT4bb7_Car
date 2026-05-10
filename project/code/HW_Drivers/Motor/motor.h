#ifndef __MOTOR_H
#define __MOTOR_H

// 电机1 - 左前 (Left Front)
#define MOTOR_M2_DIR        (P10_3)
#define MOTOR_M2_PWM        (TCPWM_CH30_P10_2)
#define MOTOR_M2_INVERT     0       // 0=不反转, 1=反转

// 电机2 - 右前 (Right Front)
#define MOTOR_M1_DIR        (P09_1)
#define MOTOR_M1_PWM        (TCPWM_CH24_P09_0)
#define MOTOR_M1_INVERT     1       // 0=不反转, 1=反转

// 电机3 - 左后 (Left Rear)
#define MOTOR_M4_DIR        (P05_3)
#define MOTOR_M4_PWM        (TCPWM_CH11_P05_2)
#define MOTOR_M4_INVERT     1       // 0=不反转, 1=反转

// 电机4 - 右后 (Right Rear)
#define MOTOR_M3_DIR        (P05_1)
#define MOTOR_M3_PWM        (TCPWM_CH09_P05_0)
#define MOTOR_M3_INVERT     0       // 0=不反转, 1=反转

#define MOTOR_PWM_MAX           5000        // PWM最大值
/**
 * @brief  初始化四轮电机
 */
void mecanum_motor_init(void);

/*
------------------------------------------------------------------------------------------------------------------
函数简介     pwm限幅
参数说明     
返回参数     
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
int16_t speed_limit(int16_t speed);

/**
 * @brief  设置单个电机速度
 * @param  dir_pin: 方向控制引脚
 * @param  pwm_ch: PWM通道
 * @param  speed: 速度值（正值正转，负值反转）
 * @param  invert: 是否反转方向 (1=反转, 0=不反转)
 */
void motor_set_single(gpio_pin_enum dir_pin, pwm_channel_enum pwm_ch, int16_t speed, uint8_t invert);

/**
 * @brief  设置电机1（左前）速度
 */
void motor_m1_set_speed(int16_t speed);

/**
 * @brief  设置电机2（右前）速度
 */
void motor_m2_set_speed(int16_t speed);

/**
 * @brief  设置电机3（左后）速度
 */
void motor_m3_set_speed(int16_t speed);

/**
 * @brief  设置电机4（右后）速度
 */
void motor_m4_set_speed(int16_t speed);

/**
 * @brief  同时设置四轮电机速度
 */
void mecanum_motor_set_all(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
/**
 * @brief  停止所有电机
 */
void mecanum_motor_stop(void);
#endif

