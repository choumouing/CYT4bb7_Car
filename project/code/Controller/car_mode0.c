/* Mode0：手动遥控模式
 * 数据流：遥控器 → wireless_control → car_forward/strafe/rotate_target
 * 输出单位：编码器周期计数（forward/strafe）和rad/s（rotate）
 */
#include "car_mode.h"


void car_mode0_init(void)
{
    car_mode0_reset();
}

void car_mode0_reset(void)
{
    /* Mode0无状态需要重置 */
}

/* 读取遥控器数据写入全局目标
 * forward_speed/strafe_speed：编码器计数（int转float）
 * rotate_speed：rad/s（float）
 * 注意：遥控器未运行时清零（双重保险）
 */
void car_mode0_update_25HZ(uint32 now_ms)
{
    const wireless_control_state_t *remote;

    (void)now_ms;

    remote = wireless_control_get_state();
    if(0U == car_start_sbus_is_running())
    {
        car_forward_target = 0.0f;
        car_strafe_target = 0.0f;
        car_rotate_target = 0.0f;
        return;
    }

    car_forward_target = (float)remote->forward_speed;
    car_strafe_target = (float)remote->strafe_speed;
    car_rotate_target = remote->rotate_speed;
}
