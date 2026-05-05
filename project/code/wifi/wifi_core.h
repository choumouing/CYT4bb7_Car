/*****************************************************************************
 * 文件: wifi_core.h
 * 模块: WiFi SPI 顶层入口
 * 职责: 统一初始化与轮询 WiFi SPI 文本命令、IMU 校准命令和 JustFloat 遥测链路
 *****************************************************************************/

#ifndef WIFI_CORE_H
#define WIFI_CORE_H

#include "zf_common_headfile.h"

/*
 * 函数功能: 初始化 WiFi SPI 功能栈。
 * 输入参数: 无。
 * 返回值: 无。
 */
void wifi_core_Init(void);

/*
 * 函数功能: 主循环轮询 WiFi SPI 链路。
 * 输入参数: 无。
 * 返回值: 无。
 */
void wifi_core_Poll(void);

/*
 * 函数功能: 根据当前车辆运行状态更新待机遥测上下文。
 * 输入参数: 无。
 * 返回值: 无。
 */
void wifi_core_UpdateStandbyContext(void);

/*
 * 函数功能: 查询 WiFi SPI 链路是否已就绪。
 * 输入参数: 无。
 * 返回值: 1-就绪，0-未就绪。
 */
uint8_t wifi_core_IsReady(void);

#endif /* WIFI_CORE_H */
