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
#ifndef CAR_START_SBUS_H
#define CAR_START_SBUS_H

#include "zf_common_headfile.h"

typedef enum
{
    CAR_START_SBUS_STATE_INIT = 0,
    CAR_START_SBUS_STATE_STANDBY,
    CAR_START_SBUS_STATE_RUNNING
} car_start_sbus_state_e;

extern car_start_sbus_state_e g_car_start_sbus_state;

void car_start_sbus_init(void);
void car_start_sbus_reset(void);
void car_start_sbus_update_25HZ(void);
car_start_sbus_state_e car_start_sbus_get_state(void);
uint8 car_start_sbus_get_mode(void);
uint8 car_start_sbus_is_running(void);
uint8 car_start_sbus_emergency_stop_active(void);

#endif /* CAR_START_SBUS_H */
