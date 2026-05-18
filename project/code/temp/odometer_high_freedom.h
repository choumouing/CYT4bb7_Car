#ifndef ODOMETER_HIGH_FREEDOM_H
#define ODOMETER_HIGH_FREEDOM_H

#include <stdint.h>

/* 高自由度里程计输出，单位为 m。
 * forward 前进为正，strafe 左移为正。 */
typedef struct
{
    float forward_distance_m;
    float strafe_distance_m;
    float travel_distance_m;
} odometer_high_freedom_output_t;

typedef struct
{
    odometer_high_freedom_output_t output;
    float yaw_rad;
    float gyro_z_sum_dps;
    float last_gyro_z_dps;
    uint16_t gyro_z_sample_count;
    uint16_t startup_hold_ticks;
} odometer_high_freedom_t;

void odometer_high_freedom_init(odometer_high_freedom_t *odometer);
void odometer_high_freedom_reset(odometer_high_freedom_t *odometer);

/* 推荐在 1000Hz IMU 更新后调用，把当前 1ms 角速度样本累加起来。
 * 100Hz 更新会优先使用最近 10ms 的均值；如果没有累加样本，则退回使用
 * odometer_high_freedom_update_100hz() 传入的 gyro_z_dps。 */
void odometer_high_freedom_accumulate_gyro_z_1000hz(odometer_high_freedom_t *odometer,
                                                    float gyro_z_dps);

void odometer_high_freedom_update_100hz(odometer_high_freedom_t *odometer,
                                        float left_front_count,
                                        float right_front_count,
                                        float left_rear_count,
                                        float right_rear_count,
                                        float gyro_z_dps);

const odometer_high_freedom_output_t *odometer_high_freedom_get_output(
    const odometer_high_freedom_t *odometer);

#endif /* ODOMETER_HIGH_FREEDOM_H */
