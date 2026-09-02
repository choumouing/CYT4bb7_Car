/*
 * 本文件属于第21届全国大学生智能汽车竞赛飞跃赛区全国冠军团队的开源代码。
 *
 * 代码总仓库：
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 *
 * 作者/维护者：杭电张跃哲
 * 作者主页：https://github.com/ZhangStudyLife/
 *
 * 本项目代码遵循 GNU GPL v3.0 或更高版本。
 * 转载、修改或再发布时，请保留本声明、作者署名和仓库链接，
 * 并按照许可证要求标明修改内容。
 *
 * 本文件中的第三方代码，其版权和许可证以原始声明及对应目录的 LICENSE 为准。
 */
#ifndef _BEACON_CONFIG_H_
#define _BEACON_CONFIG_H_

#include "zf_common_headfile.h"

#define BEACON_CONFIG_BEACON_COUNT (6U)
#define BEACON_CONFIG_FLASH_PAGE   (70U) /* Work Flash 第70页，保存地图配置 */

typedef struct
{
    float x;                        /* 全局 X 坐标 [m]，正=右侧 */
    float y;                        /* 全局 Y 坐标 [m]，正=前方 */
} beacon_config_point_t;

typedef struct
{
    float initial_position[2];
    beacon_config_point_t beacons[BEACON_CONFIG_BEACON_COUNT];
} beacon_config_data_t;

void beacon_config_init(void);
void beacon_config_reset(void);
uint16 beacon_config_get_count(void);
uint8 beacon_config_get_beacon(uint16 index, beacon_config_point_t *beacon);
void beacon_config_get_initial_position(float position[2]);
void beacon_config_get_predata(beacon_config_data_t *data);
/* 保存可编辑地图配置。 */
uint8 beacon_config_save_predata(const beacon_config_data_t *data);

#endif
