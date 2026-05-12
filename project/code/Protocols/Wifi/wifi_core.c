/**
 * @file wifi_core.c
 * @brief WiFi SPI 顶层入口实现
 *
 * 初始化顺序：wifi_cmd（建链路）→ wifi_justfloat（遥测）
 * 轮询时只调 wifi_cmd_Poll()，因为遥测发送在 100Hz 更新中触发
 */

#include "wifi_core.h"

static uint8_t s_wifi_core_inited = 0U;  /* 防重复初始化 */

/**
 * @brief 初始化 WiFi SPI 功能栈
 * 调用时机：系统启动时调一次
 * 流程：初始化命令链路 → 初始化遥测 → 开启待机遥测
 */
void wifi_core_Init(void)
{
    if (0U != s_wifi_core_inited)
    {
        return;
    }

    wifi_cmd_Init();                                /* 建立 WiFi UDP 链路 */
    wifi_justfloat_Init();                          /* 初始化遥测模块 */
    wifi_justfloat_SetStandbyUserEnable(1U);        /* 默认开启待机遥测 */
    s_wifi_core_inited = 1U;
}

/**
 * @brief 主循环轮询
 * 推进 WiFi 命令的非阻塞发送状态机
 */
void wifi_core_Poll(void)
{
    if (0U == s_wifi_core_inited)
    {
        return;
    }

    wifi_cmd_Poll();
}

/**
 * @brief 更新待机遥测上下文
 * 调用时机：车辆状态变化时
 * 传入 0 表示非待机态（遥测不受抑制）
 */
void wifi_core_UpdateStandbyContext(void)
{
    wifi_justfloat_SetStandbyContext(0U);
}

/**
 * @brief 查询 WiFi 链路是否就绪
 * @return 1=就绪（UDP 已建连），0=未就绪
 */
uint8_t wifi_core_IsReady(void)
{
    return wifi_cmd_IsReady();
}
