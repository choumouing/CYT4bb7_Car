#include "car_mode.h"
#include "car_loop.h"

/* 模式切换检测 */
static car_mode_e s_car_mode = CAR_MODE_0;           // 当前模式
static car_mode_e s_last_car_mode = CAR_MODE_0;      // 上次模式（检测变化）
static uint8 s_last_control_enabled = 0U;             // 上次控制使能状态

/* 重置所有模式状态 + 串级控制
 * 调用时机：模式切换、控制使能变化
 */
static void car_mode_reset_all(void)
{
    car_mode0_reset();
    car_mode1_reset();
    car_mode2_reset();
    Control_Reset();
}

/* 模式切换处理（25HZ）
 * 检测模式变化或控制使能变化时，重置所有状态
 * 这样切模式时PID积分、目标点标记等都会清零
 */
static void car_mode_handle_transition_25HZ(car_mode_e mode, uint8 control_enabled)
{
    uint8 need_reset = 0U;

    if(s_last_car_mode != mode)
    {
        need_reset = 1U;
    }

    if(s_last_control_enabled != control_enabled)
    {
        need_reset = 1U;
    }

    if(0U != need_reset)
    {
        car_mode_reset_all();
    }

    s_last_car_mode = mode;
    s_last_control_enabled = control_enabled;
}

/* 初始化所有模式 + 加载默认目标点 + 重置 */
void car_mode_init(void)
{
    car_mode0_init();
    car_mode1_init();
    car_mode2_init();
    car_mode2_load_default_targets();
    car_mode_reset();
}

/* 完全重置：清零所有目标、停机、标记紧急停止 */
void car_mode_reset(void)
{
    s_car_mode = CAR_MODE_0;
    s_last_car_mode = CAR_MODE_0;
    s_last_control_enabled = 0U;
    car_control_enabled = 0U;
    car_emergency_stop_active = 1U;
    car_forward_target = 0.0f;
    car_strafe_target = 0.0f;
    car_mode_reset_all();
}

/* 获取当前模式 */
car_mode_e car_mode_get(void)
{
    return s_car_mode;
}

/* 模式更新主入口（25HZ）
 * 调用链：car_loop_25HZ → 此函数
 * 数据流：
 *   1. 读取遥控器模式和控制使能（car_start_sbus）
 *   2. 检测切换 → 重置所有状态
 *   3. 控制未使能 → 清零输出直接返回（安全）
 *   4. 按模式分发到各mode_update
 *   5. mode_update内部只写car_forward/strafe_target，yaw由控制层锁0
 *   6. 100HZ的speed_loop读取这些target做控制
 */
void car_mode_update_25HZ(uint32 now_ms)
{
    /* 飞机遥控器ch4作为总开关 */
    s_car_mode = CAR_MODE_0;
    car_control_enabled = (g_air_crsf_std_ch4 >= 0.5f) ? 1U : 0U;
    car_emergency_stop_active = (car_control_enabled == 0U) ? 1U : 0U;
    car_mode_handle_transition_25HZ(s_car_mode, car_control_enabled);

    if(0U == car_control_enabled)
    {
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        return;
    }

    car_mode0_update_25HZ(now_ms);
}
