/*****************************************************************************
 * 文件: wifi_core.c
 * 模块: WiFi SPI 顶层入口
 * 职责: 聚合 WiFi SPI 初始化、主循环轮询、JustFloat 状态和 IMU 校准文本回调
 *****************************************************************************/

#include "wifi_core.h"

#include "menu/menu_config.h"
#include "wifi/wifi_cal_imu/wifi_cal_imu.h"
#include "wifi/wifi_cmd/wifi_cmd.h"
#include "wifi/wifi_justfloat/wifi_justfloat.h"

static uint8_t s_wifi_core_inited = 0U;

void wifi_core_Init(void)
{
    if (0U != s_wifi_core_inited)
    {
        return;
    }

    wifi_cmd_Init();
    wifi_justfloat_Init();
    wifi_justfloat_SetStandbyUserEnable(1U);
    wifi_cal_imu_Init();
    wifi_core_UpdateStandbyContext();

    s_wifi_core_inited = 1U;
}

void wifi_core_Poll(void)
{
    if (0U == s_wifi_core_inited)
    {
        return;
    }

    wifi_core_UpdateStandbyContext();
    wifi_cmd_Poll();
}

void wifi_core_UpdateStandbyContext(void)
{
    wifi_justfloat_SetStandbyContext((is_car_running < 0.5f) ? 1U : 0U);
}

uint8_t wifi_core_IsReady(void)
{
    return wifi_cmd_IsReady();
}
