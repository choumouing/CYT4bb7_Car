/**
 * @file wifi_cal_imu.h
 * @brief WiFi IMU 校准命令转接模块
 *
 * 功能：解析上位机通过 WiFi 发来的 imu 文本命令，转接到 IMU 校准接口
 * 命令族：
 *   imu help              - 帮助
 *   imu status            - 查询校准状态
 *   imu start gyro        - 启动角速度校准
 *   imu start accel       - 启动加速度自动椭球校准
 *   imu start accel_man   - 启动加速度手动椭球校准
 *   imu acc collect       - 手动校准采点
 *   imu acc stop          - 手动校准停止求解
 *   imu load/save/clear   - Flash 读写擦
 *   imu flash             - 读取 Flash 参数并输出
 *
 * 回包格式：OK imu xxx（成功）/ ERR imu reason（失败）
 * 限制：写操作仅待机且未解锁时允许
 */

#include "zf_common_headfile.h"
#ifndef WIFI_CAL_IMU_H
#define WIFI_CAL_IMU_H



/*
 * 函数功能: 初始化 WiFi IMU 校准命令模块，并注册校准文本回传回调
 * 输入参数: 无
 * 返回值: 无
 */
void wifi_cal_imu_Init(void);

/*
 * 函数功能: 处理一条以 imu 开头的完整文本命令
 * 输入参数:
 *   line - 完整命令文本，函数内部允许原地切分
 * 返回值: 无
 */
void wifi_cal_imu_ProcessLine(char *line);

#endif /* WIFI_CAL_IMU_H */
