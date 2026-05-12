/* 小车启动状态机 + 模式选择模块
 *
 * 数据来源：wireless_control_get_state()（遥控器解析结果）
 * 输出：car_mode_e供car_mode_update_25HZ使用
 * 安全逻辑：上电默认紧急停，遥控器显式发出启动命令才进入RUNNING
 */
#include "car_start_sbus.h"


car_start_sbus_state_e g_car_start_sbus_state = CAR_START_SBUS_STATE_INIT;

static car_mode_e s_car_mode = CAR_MODE_0;       // 当前模式
static uint8 s_emergency_stop_active = 1U;       // 紧急停车标志（默认停车）

/* 刷新模式选择
 * 当前逻辑：uwb_follow_requested=1 → Mode1，否则 → Mode0
 * mode_request_valid=0时强制Mode0
 */
static void car_start_sbus_refresh_mode(void)
{
    const wireless_control_state_t *remote = wireless_control_get_state();

    if(0U == remote->mode_request_valid)
    {
        s_car_mode = CAR_MODE_0;
        return;
    }

    if(0U != remote->uwb_follow_requested)
    {
        s_car_mode = CAR_MODE_1;
    }
    else
    {
        s_car_mode = CAR_MODE_0;
    }
}

void car_start_sbus_init(void)
{
    car_start_sbus_reset();
}

void car_start_sbus_reset(void)
{
    g_car_start_sbus_state = CAR_START_SBUS_STATE_INIT;
    s_car_mode = CAR_MODE_0;
    s_emergency_stop_active = 1U;
}

/* 25HZ状态机更新
 * 调用链：car_loop_25HZ → 此函数
 * 状态转换：
 *   INIT → STANDBY（立即）
 *   STANDBY → RUNNING：遥控器control_enabled=1且mode_request_valid=1
 *   RUNNING → STANDBY：遥控器关闭控制或模式请求失效
 * 每次都更新紧急停车标志（透传遥控器）
 */
void car_start_sbus_update_25HZ(void)
{
    const wireless_control_state_t *remote = wireless_control_get_state();

    s_emergency_stop_active = remote->emergency_stop_active;

    switch(g_car_start_sbus_state)
    {
    case CAR_START_SBUS_STATE_INIT:
        g_car_start_sbus_state = CAR_START_SBUS_STATE_STANDBY;
        break;

    case CAR_START_SBUS_STATE_STANDBY:
        if((0U != remote->control_enabled) && (0U != remote->mode_request_valid))
        {
            car_start_sbus_refresh_mode();
            g_car_start_sbus_state = CAR_START_SBUS_STATE_RUNNING;
        }
        else
        {
            s_car_mode = CAR_MODE_0;
        }
        break;

    case CAR_START_SBUS_STATE_RUNNING:
        if((0U == remote->control_enabled) || (0U == remote->mode_request_valid))
        {
            s_car_mode = CAR_MODE_0;
            g_car_start_sbus_state = CAR_START_SBUS_STATE_STANDBY;
        }
        else
        {
            car_start_sbus_refresh_mode();
        }
        break;

    default:
        car_start_sbus_reset();
        break;
    }
}

car_start_sbus_state_e car_start_sbus_get_state(void)
{
    return g_car_start_sbus_state;
}

car_mode_e car_start_sbus_get_mode(void)
{
    return s_car_mode;
}

uint8 car_start_sbus_is_running(void)
{
    return (CAR_START_SBUS_STATE_RUNNING == g_car_start_sbus_state) ? 1U : 0U;
}

uint8 car_start_sbus_emergency_stop_active(void)
{
    return s_emergency_stop_active;
}
