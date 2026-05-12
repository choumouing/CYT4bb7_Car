/* Air参数远程同步模块 - 头文件
 *
 * 功能：管理飞机端（Air）的可调参数，通过AirComm无线同步
 * 工作方式：
 *   1. 菜单修改参数 → 标记dirty
 *   2. 100HZ轮询发现dirty → 发送set_param命令给Air
 *   3. Air回复ACK → 清除dirty标记
 * 安全条件：Air在线 且 小车未运行（control_enabled=0或紧急停）时才允许编辑
 * Flash存档：支持4个存档槽（slot 0-3），独立于车端参数存档
 */

#ifndef MENU_AIR_SUPPORT_H
#define MENU_AIR_SUPPORT_H

#include "zf_common_headfile.h"

/* Air参数配置结构体 */
typedef struct
{
    char name[16];          // 参数名（用于AirComm传输）
    float *variable;        // 参数变量指针
    float step;             // 编辑步进值
    float min_val;          // 最小值
    float max_val;          // 最大值
} menu_air_param_config_t;

/* Air参数同步状态（供诊断页读取） */
typedef struct
{
    uint8 dirty_count;          // 待同步参数数量
    uint8 sending;              // 是否正在发送
    uint8 active_index;         // 当前正在同步的参数索引
    uint8 last_failed_index;    // 上次同步失败的索引
    uint8 last_result;          // 上次ACK结果
    uint8 last_status;          // 上次ACK状态码
    uint32 send_count;          // 总发送次数
    uint32 ok_count;            // 成功次数
    uint32 fail_count;          // 失败次数
} menu_air_sync_status_t;

/* Air参数变量（菜单可调） */
extern float air_min_area;      // 最小检测面积（像素）
extern float air_hold_ms;       // 保持时间（ms）
extern float air_x_bias;        // X轴偏移补偿（像素）
extern float air_y_bias;        // Y轴偏移补偿（像素）

void menu_air_support_init(void);                                                       // 初始化（注册默认参数+加载slot0）
void menu_register_param_air(const char *name, float *var, float step, float min, float max);  // 注册Air参数
uint8 menu_get_air_param_count(void);                                                   // 获取参数数量
float menu_get_air_param_by_index(uint8 index);                                         // 按索引读取
uint8 menu_set_air_param_by_index(uint8 index, float value);                            // 按索引设置（自动标记dirty）
const menu_air_param_config_t *menu_get_air_param_config(uint8 index);                  // 获取参数配置
uint8 menu_is_air_connected(void);                                                      // Air是否在线
uint8 menu_can_edit_air_params(void);                                                   // 是否允许编辑（在线且未运行）
uint8 menu_sync_all_air_params(void);                                                   // 标记所有参数为dirty（触发全量同步）
void menu_air_update_100HZ(void);                                                       // 100HZ同步轮询
void menu_get_air_sync_status(menu_air_sync_status_t *status);                          // 获取同步状态
uint8 menu_load_air_slot(uint8 slot);                                                   // 从Flash加载Air参数存档
uint8 menu_save_air_slot(uint8 slot);                                                   // 保存Air参数到Flash

#endif
