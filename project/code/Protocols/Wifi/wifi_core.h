/**
 * @file wifi_core.h
 * @brief WiFi SPI 顶层入口
 *
 * 功能：统一管理 WiFi SPI 的初始化和轮询
 * 子模块：
 *   - wifi_cmd:   文本命令收发（UDP，给上位机发命令/接收命令）
 *   - wifi_justfloat: JustFloat 二进制遥测（VOFA+ 可视化）
 *   - wifi_cal_imu:   IMU 校准命令（通过 wifi_cmd 路由）
 *
 * 调用顺序：
 *   1. wifi_core_Init() 初始化
 *   2. 主循环调 wifi_core_Poll() 推进发送状态机
 *   3. 上层通过 wifi_core_IsReady() 判断链路是否可用
 */

#include "zf_common_headfile.h"
#ifndef WIFI_CORE_H
#define WIFI_CORE_H

void wifi_core_Init(void);             /* 初始化 WiFi SPI 功能栈 */
void wifi_core_Poll(void);             /* 主循环轮询 WiFi SPI 链路 */
void wifi_core_UpdateStandbyContext(void); /* 更新待机遥测上下文 */
uint8_t wifi_core_IsReady(void);       /* 查询 WiFi SPI 链路是否已就绪：1=就绪 */

#endif /* WIFI_CORE_H */
