/* Mode2 默认目标点配置 - 头文件
 * 提供硬编码的测试路径，供car_mode_init加载
 */

#include "zf_common_headfile.h"
#ifndef CAR_MODE2_CONFIG_H
#define CAR_MODE2_CONFIG_H


/* 加载默认目标点（硬编码测试路径）
 * 调用时机：car_mode_init时自动调用
 * 坐标系：strafe=右为正(m)，forward=前为正(m)
 */
void car_mode2_load_default_targets(void);

#endif
