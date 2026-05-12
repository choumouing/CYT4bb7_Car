#include "zf_common_headfile.h"
#ifndef _BEACON_DETECTION_H_
#define _BEACON_DETECTION_H_



#ifdef __cplusplus
extern "C" {
#endif

/* 碰撞位置枚举（信标检测结果） */
typedef enum
{
    BEACON_BUMP_LOCATION_UNKNOWN = 0,
    BEACON_BUMP_LOCATION_FRONT,          /* 正前方 */
    BEACON_BUMP_LOCATION_REAR,           /* 正后方 */
    BEACON_BUMP_LOCATION_LEFT,           /* 正左方 */
    BEACON_BUMP_LOCATION_RIGHT,          /* 正右方 */
    BEACON_BUMP_LOCATION_LEFT_FRONT,     /* 左前方 */
    BEACON_BUMP_LOCATION_RIGHT_FRONT,    /* 右前方 */
    BEACON_BUMP_LOCATION_LEFT_REAR,      /* 左后方 */
    BEACON_BUMP_LOCATION_RIGHT_REAR,     /* 右后方 */
    BEACON_BUMP_LOCATION_DIAGONAL_LF_RR, /* 对角线 左前-右后 */
    BEACON_BUMP_LOCATION_DIAGONAL_RF_LR  /* 对角线 右前-左后 */
} beacon_bump_location_t;

/* 碰撞置信度 */
typedef enum
{
    BEACON_BUMP_CONFIDENCE_NONE = 0,  /* 无碰撞 */
    BEACON_BUMP_CONFIDENCE_LOW,       /* 低置信（可能是手推等轻触） */
    BEACON_BUMP_CONFIDENCE_HIGH       /* 高置信（明确碰撞） */
} beacon_bump_confidence_t;

/* 轮子位掩码，用于标记哪几个轮子感受到了碰撞 */
#define BEACON_BUMP_WHEEL_LF_MASK (0x01U) /* 左前轮 */
#define BEACON_BUMP_WHEEL_RF_MASK (0x02U) /* 右前轮 */
#define BEACON_BUMP_WHEEL_LR_MASK (0x04U) /* 左后轮 */
#define BEACON_BUMP_WHEEL_RR_MASK (0x08U) /* 右后轮 */

/* 碰撞检测输出状态（外部只读） */
typedef struct
{
    uint8_t bump_detected;             /* 1=检测到碰撞，hold_ticks 期间保持 1 */
    uint8_t partial_bump;              /* 1=部分轮子参与（可能斜碰） */
    uint8_t wheel_mask;                /* 哪些轮子感受到碰撞，用 BEACON_BUMP_WHEEL_*_MASK */
    beacon_bump_location_t location;       /* 碰撞位置 */
    beacon_bump_confidence_t confidence;   /* 置信度 */

    uint32_t event_count;              /* 累计碰撞事件计数 */
    uint16_t hold_ticks;               /* 碰撞状态保持剩余 tick 数 */

    float score;                       /* 碰撞强度评分 */
    float speed_mps;                   /* 碰撞时车速 */
    float forward_velocity_mps;        /* 碰撞时前后速度 */
    float strafe_velocity_mps;         /* 碰撞时横向速度 */
    float gyro_xy_dps;                 /* 碰撞时 XY 平面角速率 */
    float tilt_rate_dps;               /* 碰撞时倾斜速率 */
    float tilt_deg;                    /* 碰撞时倾斜角度 */
    float accel_norm_error_g;          /* 碰撞时加速度模长偏差 */
    float wheel_highpass_count;        /* 碰撞时轮速高通值 */
} beacon_detection_state_t;

/* 摄像头追踪目标（来自 Camera SPI） */
typedef struct
{
    uint8 valid;               /* 目标是否有效 */
    uint8 camera_id;           /* 摄像头编号 */
    uint16 frame_id;           /* 帧号 */
    uint16 spot_count;         /* 当前帧检测到的光点总数 */
    uint16 spot_index;         /* 被选中光点的索引 */
    uint16 x;                  /* 目标像素 X */
    uint16 y;                  /* 目标像素 Y */
    uint16 area;               /* 光点面积 */
    uint32 age_ms;             /* 目标持续存在时间(ms) */
    uint32 last_update_ms;     /* 最后更新时间(ms) */
} beacon_camera_target_t;

extern beacon_detection_state_t g_beacon_detection;  /* 全局碰撞检测状态，其他模块可读 */

/* 上电调一次，内部调 beacon_detection_reset */
void beacon_detection_init(void);

/* 清零所有碰撞状态和滤波器，调头/脱困后可手动调 */
void beacon_detection_reset(void);

/* 100Hz 周期调用，从 IMU+编码器检测碰撞事件
 * 碰撞后 hold_ticks 内不会重复触发（防抖） */
void beacon_detection_update_100HZ(void);

/* 获取碰撞检测状态指针（只读） */
const beacon_detection_state_t *beacon_detection_get_state(void);

/* 获取摄像头追踪目标，返回 valid 标志
 * target: 输出缓冲区，调用方传入
 * 返回值: 0=无目标，非0=目标有效 */
uint8 beacon_detection_get_camera_target(beacon_camera_target_t *target);

#ifdef __cplusplus
}
#endif

#endif
