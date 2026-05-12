/* 小车启动状态机 + 模式选择模块 - 头文件
 *
 * 状态机：INIT → STANDBY → RUNNING
 *   INIT：上电初始态，立即转STANDBY
 *   STANDBY：等待遥控器发出control_enabled + mode_request
 *   RUNNING：正常运行，持续读取模式；遥控器断开或关闭控制则回STANDBY
 *
 * 模式选择：当前只支持Mode0（手动）和Mode1（UWB跟随），由遥控器uwb_follow_requested决定
 * 紧急停车：直接透传遥控器的emergency_stop_active
 */

#ifndef CAR_START_SBUS_H
#define CAR_START_SBUS_H

#include "zf_common_headfile.h"

/* 启动状态机枚举 */
typedef enum
{
    CAR_START_SBUS_STATE_INIT = 0,      // 上电初始态（瞬态）
    CAR_START_SBUS_STATE_STANDBY,       // 待机（等遥控器启动命令）
    CAR_START_SBUS_STATE_RUNNING        // 运行中（控制使能）
} car_start_sbus_state_e;

extern car_start_sbus_state_e g_car_start_sbus_state;

void car_start_sbus_init(void);                     // 初始化（reset）
void car_start_sbus_reset(void);                    // 重置到INIT + 紧急停
void car_start_sbus_update_25HZ(void);              // 25HZ状态机更新
car_start_sbus_state_e car_start_sbus_get_state(void);  // 获取当前状态
car_mode_e car_start_sbus_get_mode(void);           // 获取当前模式（由遥控器决定）
uint8 car_start_sbus_is_running(void);              // 是否处于RUNNING（1=运行中）
uint8 car_start_sbus_emergency_stop_active(void);   // 紧急停车是否激活（1=停车）

#endif /* CAR_START_SBUS_H */
