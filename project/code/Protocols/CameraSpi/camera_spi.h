/**
 * @file camera_spi.h
 * @brief 三路摄像头 SPI 通信协议层
 *
 * 主从关系：
 *   - MCU（本文件）= SPI 主机，3 个摄像头模块 = SPI 从机
 *   - 主机轮询从机（round-robin），每次与一个从机做全双工传输
 *
 * 帧格式（请求帧 / 响应帧）：
 *   [0]   HEAD1: 0xAA
 *   [1]   HEAD2: 0x55
 *   [2]   CMD:   命令字节（当前固定 0x20 = SYNC_DATA）
 *   [3:4] LEN:   payload 长度（大端 uint16）
 *   [5:N] DATA:  payload 数据
 *   [N+1:N+2] CRC16（从 CMD 开始算，大端）
 *   [N+3] TAIL:  0xED
 *   帧总开销 = 8 字节（head+cmd+len+crc+tail）
 *
 * 下行（主机→从机）payload 格式（6+12=18 字节）：
 *   [0:3] sequence: 数据包序号（uint32 LE）
 *   [4:5] length:   有效数据长度（uint16 LE）
 *   [6:17] data:    应用数据（12 字节，不足补零）
 *
 * 上行（从机→主机）payload 格式（12+12=24 字节）：
 *   [0:3] sequence:       从机自身序号（uint32 LE）
 *   [3:7] ack_sequence:   从机已确认的最新下行序号（uint32 LE）
 *   [8:9] length:         有效数据长度（uint16 LE）
 *   [10]  flags:          标志位
 *   [11]  peer_last_error:从机最近一次错误码
 *   [12:23] data:         应用数据（12 字节）
 *
 * ready 信号：从机通过 GPIO 中断通知主机"我有数据了"
 *   - 主机收到中断 → camera_spi_notify_ready() 设置 ready_mask
 *   - 轮询时 round-robin 选择 ready 的从机发起传输
 *
 * target 数据含义（从上行 data 解析）：
 *   word0[0:1]: frame_id（图像帧编号）
 *   word0[2:3]: spot_count（检测到的目标点数）
 *   word1[0:1]: spot_index（当前目标点序号）
 *   word1[2:3]: x（目标中心 X 坐标，像素）
 *   word2[0:1]: y（目标中心 Y 坐标，像素）
 *   word2[2:3]: area（目标面积，像素^2）
 *
 * 超时策略：
 *   - SPI 传输超时通过 CAMERA_SPI_POLL_TIMEOUT_COUNT 控制
 *   - 从机离线判定：上次成功通信超过阈值
 *
 * 错误处理路径：
 *   帧头不匹配 → ERR_INVALID_HEAD，从机 err_count++
 *   CRC 校验失败 → ERR_CRC，从机 err_count++
 *   payload 长度超限 → ERR_PAYLOAD_LONG
 *   帧尾不匹配 → ERR_INVALID_TAIL
 *   SPI 硬件错误 → ERR_HW
 *   传输超时 → ERR_TIMEOUT，调 abort_transfer
 */

#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"

#define CAMERA_SPI_SLAVE_COUNT              (3U)    /* 摄像头从机数量 */
#define CAMERA_SPI_APP_DATA_CAPACITY        (12U)   /* 每帧应用数据最大字节数 */

/** @brief 从机 ID 枚举 */
typedef enum
{
    CAMERA_SPI_SLAVE_1 = 0,
    CAMERA_SPI_SLAVE_2 = 1,
    CAMERA_SPI_SLAVE_3 = 2
} camera_spi_slave_id_t;

/**
 * @brief 下行应用数据缓冲区
 * 谁用：上层通过 camera_spi_set_downlink_payload() 设置要下发给从机的数据
 */
typedef struct
{
    uint16 length;                                /* 有效数据长度（≤12） */
    uint8 data[CAMERA_SPI_APP_DATA_CAPACITY];     /* 应用数据 */
} camera_spi_payload_buffer_t;

/**
 * @brief 下行 payload 结构
 * sequence 每次 set_downlink_payload 递增，从机通过 ack_sequence 告知已收到哪个
 */
typedef struct
{
    uint32 sequence;                              /* 下行序号 */
    uint16 length;                                /* 有效数据长度 */
    uint8 data[CAMERA_SPI_APP_DATA_CAPACITY];     /* 应用数据 */
} camera_spi_downlink_payload_t;

/**
 * @brief 上行 payload 结构（从机响应的内容）
 */
typedef struct
{
    uint32 sequence;                              /* 从机自身序号 */
    uint32 ack_sequence;                          /* 从机确认的最新下行序号 */
    uint16 length;                                /* 有效数据长度 */
    uint8 flags;                                  /* 标志位 */
    uint8 peer_last_error;                        /* 从机最近错误码 */
    uint8 data[CAMERA_SPI_APP_DATA_CAPACITY];     /* 应用数据（含检测结果） */
} camera_spi_uplink_payload_t;

/**
 * @brief 单个从机的通信状态
 * 谁用：上层通过 camera_spi_get_status() 读取，用于判断从机在线/离线
 */
typedef struct
{
    uint8 online;                                 /* 在线标志：1=在线 */
    uint8 int_level;                              /* ready 引脚当前电平 */
    uint8 last_error;                             /* 最近一次通信错误码 */
    uint32 ok_count;                              /* 成功通信次数 */
    uint32 err_count;                             /* 失败通信次数 */
    uint32 last_update_ms;                        /* 最近一次成功通信的系统时间 */
    camera_spi_uplink_payload_t uplink;           /* 最近一次成功的上行数据 */
} camera_spi_slave_status_t;

/**
 * @brief 目标检测结果（从上行 data 解析而来）
 * 谁用：上层通过 camera_spi_get_target() 获取当前最佳目标
 * 选择策略：从三路摄像头中选 area 最大的目标
 */
typedef struct
{
    uint8 valid;                  /* 目标有效：1=有效 */
    uint8 camera_id;              /* 来源摄像头 ID（0/1/2） */
    uint16 frame_id;              /* 图像帧编号 */
    uint16 spot_count;            /* 该帧检测到的目标总数 */
    uint16 spot_index;            /* 当前目标在列表中的序号 */
    uint16 x;                     /* 目标中心 X 坐标（像素） */
    uint16 y;                     /* 目标中心 Y 坐标（像素） */
    uint16 area;                  /* 目标面积（像素^2） */
    uint32 uplink_sequence;       /* 来源上行序号 */
    uint32 last_update_ms;        /* 最近一次更新的系统时间 */
    uint32 age_ms;                /* 数据新鲜度 = 当前时间 - last_update_ms */
} camera_spi_target_t;

/**
 * @brief 诊断统计结构体
 * 谁用：调试时通过 camera_spi_get_diag() 获取
 */
typedef struct
{
    uint8 transfer_busy;                          /* 当前是否有传输在进行 */
    uint8 active_slave;                           /* 当前活动从机 ID */
    uint8 ready_mask;                             /* 就绪从机位掩码 */
    uint8 downlink_mask;                          /* 需要下发新数据的从机位掩码 */
    uint8 last_error;                             /* 最近一次错误码 */
    uint32 poll_count;                            /* poll 调用次数 */
    uint32 transfer_start_count;                  /* 传输发起次数 */
    uint32 transfer_done_count;                   /* 传输完成次数 */
    uint32 transfer_error_count;                  /* 传输失败次数 */
    uint32 timeout_count;                         /* 传输超时次数 */
    uint32 ready_irq_count[CAMERA_SPI_SLAVE_COUNT]; /* 各从机 ready 中断计数 */
} camera_spi_diag_t;

/**
 * @brief 初始化摄像头 SPI 模块
 * 调用时机：系统启动时调一次
 * 内部：清零所有状态，初始化底层 SPI 硬件
 */
void camera_spi_init(void);

/**
 * @brief 主循环轮询摄像头 SPI
 * 调用频率：越高越好（主循环每次调）
 * 内部流程：
 *   1. 如果有传输在进行 → 检查完成/超时
 *   2. 否则 → 刷新 ready 电平 → 发起下一个从机的传输
 */
void camera_spi_poll(void);

/**
 * @brief 100Hz 更新摄像头 SPI 状态
 * @param system_time_ms 当前系统时间（ms）
 * 内部：刷新 ready 电平、刷新最佳目标选择
 */
void camera_spi_update_100HZ(uint32 system_time_ms);

/**
 * @brief 从机 ready 中断回调
 * @param id 就绪的从机 ID
 * 谁调用：GPIO 中断处理函数
 * 效果：设置 ready_mask，后续 poll 会选中该从机
 */
void camera_spi_notify_ready(camera_spi_slave_id_t id);

/**
 * @brief 获取从机通信状态
 * @param id 从机 ID
 * @param status 输出状态结构体
 * @return CAMERA_SPI_ERR_OK 成功，其他为错误码
 */
uint8 camera_spi_get_status(camera_spi_slave_id_t id, camera_spi_slave_status_t *status);

/**
 * @brief 获取诊断统计
 * @param diag 输出诊断结构体
 * @return CAMERA_SPI_ERR_OK 成功
 */
uint8 camera_spi_get_diag(camera_spi_diag_t *diag);

/**
 * @brief 获取当前最佳目标检测结果
 * @param target 输出目标结构体
 * @return 1=有效目标，0=无有效目标
 */
uint8 camera_spi_get_target(camera_spi_target_t *target);

/**
 * @brief 设置下行应用数据
 * @param payload 要下发的数据
 * 谁调用：上层模块，设置后会标记所有从机需要接收新数据
 * 注意：每次调用 sequence 自增，从机通过 ack_sequence 告知同步状态
 */
void camera_spi_set_downlink_payload(const camera_spi_payload_buffer_t *payload);

#endif
