#ifndef CAR_MODE_H
#define CAR_MODE_H

/* 小车运行模式枚举
 * 切换来源：遥控器mode_request
 * Mode0: 手动遥控（前后左右+旋转）
 * Mode1: UWB跟随模式（跟踪一个标签）
 * Mode2: 目标点导航模式（UWB+里程计多点巡航）
 * Mode3-8: 保留（当前走Mode0）
 */
typedef enum
{
    CAR_MODE_0 = 0,
    CAR_MODE_1,
    CAR_MODE_2,
    CAR_MODE_3,
    CAR_MODE_4,
    CAR_MODE_5,
    CAR_MODE_6,
    CAR_MODE_7,
    CAR_MODE_8
} car_mode_e;

#include "zf_common_headfile.h"

/* UWB跟随配置 */
#define UWB_FOLLOW_PERIOD_MS              (40U)     // 跟随更新周期（ms）
#define UWB_FOLLOW_TIMEOUT_MS             (160U)    // 跟随超时（ms），超时后清零输出

/* 目标点导航配置 */
#define TARGET_FOLLOW_MAX_TARGETS              (8U)     // 最大目标点数
#define TARGET_FOLLOW_INVALID_INDEX            (0xFFU)  // 无效索引标记

/* 目标点导航子状态 */
#define TARGET_FOLLOW_MODE_IDLE                (0U)     // 空闲
#define TARGET_FOLLOW_MODE_FOLLOW_TAG          (1U)     // 跟踪标签（等同Mode1）
#define TARGET_FOLLOW_MODE_GOTO_TARGET         (2U)     // 前往目标点
#define TARGET_FOLLOW_MODE_TARGET_REACHED      (3U)     // 到达目标点

/* 目标点导航参数 */
#define TARGET_FOLLOW_TAG_GUARD_RADIUS_M       (1.00f)  // 标签保护半径（m），超出则切回跟随
#define TARGET_FOLLOW_TARGET_MATCH_RADIUS_M    (1.00f)  // 目标匹配半径（m），用于筛选候选目标
#define TARGET_FOLLOW_REACHED_RADIUS_M         (0.08f)  // 到达判定半径（m）
#define TARGET_FOLLOW_POSITION_DEADBAND_M      (0.02f)  // 位置死区（m），消除震荡
#define TARGET_FOLLOW_OUTPUT_LIMIT             (400.0f) // 输出限幅（编码器计数）
#define TARGET_FOLLOW_POS_KP                   (120.0f) // 位置环比例系数
#define TARGET_FOLLOW_POS_KI                   (0.0f)   // 位置环积分系数
#define TARGET_FOLLOW_POS_KD                   (30.0f)  // 位置环微分系数
#define TARGET_FOLLOW_POS_I_LIMIT              (0.0f)   // 位置环积分限幅
#define TARGET_FOLLOW_UWB_TIMEOUT_MS           (160U)   // UWB超时（ms）

/* Mode1（UWB跟随）运行状态
 * 供诊断页和调试读取
 */
typedef struct
{
    float raw_x_cm;         // 原始UWB X坐标（cm）
    float raw_y_cm;         // 原始UWB Y坐标（cm）
    float filt_x_cm;        // 滤波后X坐标（cm）
    float filt_y_cm;        // 滤波后Y坐标（cm）
    float error_x_cm;       // X方向死区后误差（cm）
    float error_y_cm;       // Y方向死区后误差（cm）
    float x_pid_p_term;     // X轴PID P项
    float x_pid_i_term;     // X轴PID I项
    float x_pid_d_term;     // X轴PID D项
    float y_pid_p_term;     // Y轴PID P项
    float y_pid_i_term;     // Y轴PID I项
    float y_pid_d_term;     // Y轴PID D项
    float forward_target;   // 前后输出（编码器计数）
    float strafe_target;    // 左右输出（编码器计数）
    uint8 tag_online;       // 标签是否在线（超时判断）
    uint8 output_valid;     // 输出是否有效（在线且数据正常）
} car_mode1_state_t;

/* Mode2目标点数据 */
typedef struct
{
    float strafe_m;     // 目标点X坐标（m，右为正）
    float forward_m;    // 目标点Y坐标（m，前为正）
    uint8 reached;      // 是否已到达
    uint8 valid;        // 是否有效
} car_mode2_point_t;

/* Mode2（目标点导航）运行状态
 * 坐标系：全局坐标，原点为里程计起点
 * 切换逻辑：标签在线→跟随标签；找到候选→前往目标；到达→标记完成
 */
typedef struct
{
    car_mode2_point_t targets[TARGET_FOLLOW_MAX_TARGETS];  // 目标点列表
    float car_strafe_m;             // 当前车X坐标（m）
    float car_forward_m;            // 当前车Y坐标（m）
    float tag_strafe_m;             // 标签全局X坐标（m）
    float tag_forward_m;            // 标签全局Y坐标（m）
    float tag_relative_strafe_m;    // 标签相对车X偏移（m）
    float tag_relative_forward_m;   // 标签相对车Y偏移（m）
    float car_tag_distance_m;       // 车到标签距离（m）
    float target_tag_distance_m;    // 候选目标到标签距离（m）
    float target_car_distance_m;    // 当前位置到目标距离（m）
    float target_error_strafe_m;    // 到目标X误差（m，全局）
    float target_error_forward_m;   // 到目标Y误差（m，全局）
    float target_pid_strafe_output; // 目标导航X输出（编码器计数）
    float target_pid_forward_output;// 目标导航Y输出（编码器计数）
    float forward_target;           // 最终前后输出（编码器计数）
    float strafe_target;            // 最终左右输出（编码器计数）
    uint8 target_count;             // 目标点数量
    uint8 active_index;             // 当前活跃目标索引
    uint8 candidate_index;          // 候选目标索引
    uint8 mode;                     // 子状态（IDLE/FOLLOW/GOTO/REACHED）
    uint8 tag_online;               // 标签在线
    uint8 car_in_tag_range;         // 车在标签保护半径内
    uint8 target_in_tag_range;      // 候选目标在标签保护半径内
    uint8 output_valid;             // 输出有效
} car_mode2_state_t;

/* 全局运行状态（供诊断页读取） */
extern car_mode1_state_t g_car_mode1_state;
extern car_mode2_state_t g_car_mode2_state;

/* 模式管理接口 */
void car_mode_init(void);                         // 初始化所有模式
void car_mode_reset(void);                        // 重置到Mode0 + 停机
car_mode_e car_mode_get(void);                    // 获取当前模式
void car_mode_update_25HZ(uint32 now_ms);         // 25HZ更新入口（模式分发）

/* Mode0：手动遥控 */
void car_mode0_init(void);
void car_mode0_reset(void);
void car_mode0_update_25HZ(uint32 now_ms);        // 读取遥控器→写car_forward/strafe/rotate_target

/* Mode1：UWB跟随 */
void car_mode1_init(void);
void car_mode1_reset(void);
void car_mode1_update_25HZ(uint32 now_ms);        // UWB数据→PID→写car_forward/strafe_target

/* Mode2：目标点导航 */
void car_mode2_init(void);
void car_mode2_reset(void);
void car_mode2_update_25HZ(uint32 now_ms);        // 里程计+UWB→找目标→PID→写输出
void car_mode2_restart_targets(void);             // 重置所有目标到达标记
void car_mode2_clear_targets(void);               // 清空所有目标
uint8 car_mode2_add_target(float strafe_m, float forward_m);       // 添加目标点
uint8 car_mode2_set_target(uint8 index, float strafe_m, float forward_m); // 设置指定目标点

/* 加载默认目标点（硬编码的测试路径） */
void car_mode2_load_default_targets(void);

#endif /* CAR_MODE_H */
