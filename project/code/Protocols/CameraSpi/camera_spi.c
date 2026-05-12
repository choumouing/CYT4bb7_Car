/**
 * @file camera_spi.c
 * @brief 三路摄像头 SPI 通信实现
 *
 * 状态机概要：
 *   poll() → transfer_busy?
 *     ├── 是 → hw_transfer_finished? → 完成处理 / 超时处理
 *     └── 否 → 刷新 ready 电平 → pick_next_slave → build_request → hw_start_transfer
 *
 * round-robin 调度：ready_mask | downlink_mask 中按 round_robin_next 循环选取
 */

#include "camera_spi.h"

/* ===== 帧格式常量 ===== */
#define CAMERA_SPI_FRAME_HEAD_1             (0xAAU)   /* 帧头字节1 */
#define CAMERA_SPI_FRAME_HEAD_2             (0x55U)   /* 帧头字节2 */
#define CAMERA_SPI_FRAME_TAIL               (0xEDU)   /* 帧尾 */
#define CAMERA_SPI_CMD_SYNC_DATA            (0x20U)   /* 同步数据命令 */
#define CAMERA_SPI_FRAME_OVERHEAD           (8U)      /* 帧开销：head(2)+cmd(1)+len(2)+crc(2)+tail(1) */
#define CAMERA_SPI_MAX_DATA_SIZE            (100U)    /* payload 最大长度 */

/* payload 尺寸 = 元数据 + 应用数据 */
#define CAMERA_SPI_REQ_META_SIZE            (6U)      /* 下行元数据：sequence(4)+length(2) */
#define CAMERA_SPI_RESP_META_SIZE           (12U)     /* 上行元数据：sequence(4)+ack(4)+length(2)+flags(1)+err(1) */
#define CAMERA_SPI_REQ_PAYLOAD_SIZE         (CAMERA_SPI_REQ_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)
#define CAMERA_SPI_RESP_PAYLOAD_SIZE        (CAMERA_SPI_RESP_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)

/* 完整帧长度 = 帧开销 + payload */
#define CAMERA_SPI_REQ_FRAME_LEN            (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_REQ_PAYLOAD_SIZE)
#define CAMERA_SPI_RESP_FRAME_LEN           (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_RESP_PAYLOAD_SIZE)

/* SPI 传输长度 = 取请求/响应中较大的（全双工，收发等长） */
#define CAMERA_SPI_TRANSFER_LEN             CAMERA_SPI_RESP_FRAME_LEN

/* 全从机掩码：3 个从机 = 0b111 */
#define CAMERA_SPI_ALL_SLAVE_MASK           ((uint8)((1U << CAMERA_SPI_SLAVE_COUNT) - 1U))

/* SPI 传输轮询超时计数（软件计数，非时间单位） */
#define CAMERA_SPI_POLL_TIMEOUT_COUNT       (50000U)

/* ===== 错误码 ===== */
#define CAMERA_SPI_ERR_OK                   (0U)      /* 成功 */
#define CAMERA_SPI_ERR_FRAME_SHORT          (1U)      /* 收到数据太短，不足帧头 */
#define CAMERA_SPI_ERR_INVALID_HEAD         (2U)      /* 帧头不匹配 */
#define CAMERA_SPI_ERR_PAYLOAD_LONG         (3U)      /* payload 长度超限 */
#define CAMERA_SPI_ERR_INVALID_TAIL         (4U)      /* 帧尾不匹配 */
#define CAMERA_SPI_ERR_CRC                  (5U)      /* CRC 校验失败 */
#define CAMERA_SPI_ERR_INCOMPLETE           (6U)      /* 帧不完整（长度不足） */
#define CAMERA_SPI_ERR_NULL_PTR             (7U)      /* 空指针 */
#define CAMERA_SPI_ERR_TIMEOUT              (8U)      /* SPI 传输超时 */
#define CAMERA_SPI_ERR_DATA_SIZE            (9U)      /* 数据尺寸不匹配 */
#define CAMERA_SPI_ERR_INVALID_SLAVE        (10U)     /* 无效从机 ID */
#define CAMERA_SPI_ERR_INVALID_CMD          (11U)     /* 命令字节不匹配 */
#define CAMERA_SPI_ERR_HW                   (12U)     /* SPI 硬件错误 */

/**
 * @brief 主机状态结构体
 */
typedef struct
{
    uint8 transfer_busy;                          /* 1=有传输在进行 */
    uint8 active_slave;                           /* 当前活动从机 ID */
    uint8 round_robin_next;                       /* 下次轮询从机 ID（round-robin 游标） */
    volatile uint8 ready_mask;                    /* 就绪从机位掩码（中断可能修改） */
    uint8 downlink_mask;                          /* 需要下发数据的从机位掩码 */
    uint32 active_downlink_sequence;              /* 当前传输使用的下行序号 */
    uint32 transfer_poll_guard;                   /* 传输轮询超时计数器 */
    uint8 tx_buf[CAMERA_SPI_TRANSFER_LEN];        /* SPI 发送缓冲区 */
    uint8 rx_buf[CAMERA_SPI_TRANSFER_LEN];        /* SPI 接收缓冲区 */
} camera_spi_master_state_t;

/* ===== 静态状态 ===== */
static camera_spi_master_state_t s_camera_spi_master;
static camera_spi_slave_status_t s_camera_spi_status[CAMERA_SPI_SLAVE_COUNT];
static camera_spi_downlink_payload_t s_camera_spi_downlink;
static uint32 s_camera_spi_downlink_synced[CAMERA_SPI_SLAVE_COUNT];  /* 各从机已同步的下行序号 */
static camera_spi_diag_t s_camera_spi_diag;
static camera_spi_target_t s_camera_spi_target;
static uint32 s_camera_spi_time_ms;

/* ===== 工具函数 ===== */

/**
 * @brief 从机 ID 转位掩码
 * @param id 从机 ID (0/1/2)
 * @return 对应的位掩码 (1/2/4)
 */
static uint8 camera_spi_slave_mask(camera_spi_slave_id_t id)
{
    return (uint8)(1U << (uint8)id);
}

/**
 * @brief 写 uint16 大端字节序
 */
static void camera_spi_write_u16_be(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value >> 8);
    buffer[1] = (uint8)(value & 0xFFU);
}

/**
 * @brief 读 uint16 大端字节序
 */
static uint16 camera_spi_read_u16_be(const uint8 *buffer)
{
    return (uint16)(((uint16)buffer[0] << 8) | buffer[1]);
}

/**
 * @brief 写 uint16 小端字节序
 */
static void camera_spi_write_u16_le(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
}

/**
 * @brief 读 uint16 小端字节序
 */
static uint16 camera_spi_read_u16_le(const uint8 *buffer)
{
    return (uint16)(((uint16)buffer[1] << 8) | buffer[0]);
}

/**
 * @brief 写 uint32 小端字节序
 */
static void camera_spi_write_u32_le(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

/**
 * @brief 读 uint32 小端字节序
 */
static uint32 camera_spi_read_u32_le(const uint8 *buffer)
{
    return ((uint32)buffer[0]) |
           ((uint32)buffer[1] << 8) |
           ((uint32)buffer[2] << 16) |
           ((uint32)buffer[3] << 24);
}

/**
 * @brief CRC16-MODBUS 计算（多项式 0xA001，反射输入输出）
 * 注意：与 AirComm 的 CRC-CCITT 不同，这里用的是反射型算法
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC16 值
 */
static uint16 camera_spi_crc16(const uint8 *data, uint16 len)
{
    uint16 crc = 0xFFFFU;
    uint16 i;
    uint8 j;

    for(i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for(j = 0U; j < 8U; j++)
        {
            if((crc & 0x0001U) != 0U)
            {
                crc = (uint16)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

/* ===== 帧构建与解析 ===== */

/**
 * @brief 构建 SPI 帧
 *
 * 帧布局：[AA 55 CMD LEN_H LEN_D] [payload...] [CRC_L CRC_H] [ED]
 * buffer 剩余部分填 0xFF（SPI 空闲电平）
 *
 * @param cmd 命令字节
 * @param payload payload 数据
 * @param payload_len payload 长度
 * @param buffer 输出缓冲区
 * @param capacity 缓冲区总容量
 * @return 实际帧长度，0=失败
 */
static uint16 camera_spi_build_frame(uint8 cmd,
                                     const uint8 *payload,
                                     uint16 payload_len,
                                     uint8 *buffer,
                                     uint16 capacity)
{
    uint16 crc;
    uint16 frame_len = (uint16)(CAMERA_SPI_FRAME_OVERHEAD + payload_len);

    if((buffer == NULL) ||
       (payload_len > CAMERA_SPI_MAX_DATA_SIZE) ||
       (frame_len > capacity))
    {
        return 0U;
    }

    /* 先填 0xFF（SPI 空闲字节） */
    memset(buffer, 0xFF, capacity);

    /* 帧头 + 命令 + 长度 */
    buffer[0] = CAMERA_SPI_FRAME_HEAD_1;
    buffer[1] = CAMERA_SPI_FRAME_HEAD_2;
    buffer[2] = cmd;
    camera_spi_write_u16_be(&buffer[3], payload_len);

    /* 拷贝 payload */
    if((payload != NULL) && (payload_len > 0U))
    {
        memcpy(&buffer[5], payload, payload_len);
    }

    /* CRC 覆盖 cmd + len + payload */
    crc = camera_spi_crc16(&buffer[2], (uint16)(3U + payload_len));
    buffer[5U + payload_len] = (uint8)(crc & 0xFFU);
    buffer[6U + payload_len] = (uint8)(crc >> 8);
    buffer[7U + payload_len] = CAMERA_SPI_FRAME_TAIL;

    return frame_len;
}

/**
 * @brief 解析 SPI 响应帧
 *
 * 校验路径：
 *   1. rx_buf 空 → ERR_NULL_PTR
 *   2. 长度不足帧头 → ERR_FRAME_SHORT
 *   3. 帧头不匹配 → ERR_INVALID_HEAD
 *   4. 命令字节不匹配 → ERR_INVALID_CMD
 *   5. payload 长度超限 → ERR_PAYLOAD_LONG
 *   6. payload 长度与期望不符 → ERR_DATA_SIZE
 *   7. 帧不完整 → ERR_INCOMPLETE
 *   8. 帧尾不匹配 → ERR_INVALID_TAIL
 *   9. CRC 不匹配 → ERR_CRC
 *   10. 全部通过 → 拷贝 payload，返回 ERR_OK
 *
 * @param rx_buf 接收缓冲区
 * @param rx_len 接收到的字节数
 * @param expected_cmd 期望命令字节
 * @param payload 输出 payload 缓冲区
 * @param expected_payload_len 期望 payload 长度
 * @return 错误码
 */
static uint8 camera_spi_parse_frame(const uint8 *rx_buf,
                                    uint16 rx_len,
                                    uint8 expected_cmd,
                                    uint8 *payload,
                                    uint16 expected_payload_len)
{
    uint16 payload_len;
    uint16 frame_len;
    uint16 crc_calc;
    uint16 crc_recv;

    if(rx_buf == NULL)
    {
        return CAMERA_SPI_ERR_NULL_PTR;
    }

    if(rx_len < CAMERA_SPI_FRAME_OVERHEAD)
    {
        return CAMERA_SPI_ERR_FRAME_SHORT;
    }

    if((rx_buf[0] != CAMERA_SPI_FRAME_HEAD_1) || (rx_buf[1] != CAMERA_SPI_FRAME_HEAD_2))
    {
        return CAMERA_SPI_ERR_INVALID_HEAD;
    }

    if(rx_buf[2] != expected_cmd)
    {
        return CAMERA_SPI_ERR_INVALID_CMD;
    }

    payload_len = camera_spi_read_u16_be(&rx_buf[3]);
    if(payload_len > CAMERA_SPI_MAX_DATA_SIZE)
    {
        return CAMERA_SPI_ERR_PAYLOAD_LONG;
    }

    if(payload_len != expected_payload_len)
    {
        return CAMERA_SPI_ERR_DATA_SIZE;
    }

    frame_len = (uint16)(CAMERA_SPI_FRAME_OVERHEAD + payload_len);
    if(rx_len < frame_len)
    {
        return CAMERA_SPI_ERR_INCOMPLETE;
    }

    if(rx_buf[7U + payload_len] != CAMERA_SPI_FRAME_TAIL)
    {
        return CAMERA_SPI_ERR_INVALID_TAIL;
    }

    crc_calc = camera_spi_crc16(&rx_buf[2], (uint16)(3U + payload_len));
    crc_recv = (uint16)rx_buf[5U + payload_len] |
               ((uint16)rx_buf[6U + payload_len] << 8);
    if(crc_calc != crc_recv)
    {
        return CAMERA_SPI_ERR_CRC;
    }

    if((payload != NULL) && (payload_len > 0U))
    {
        memcpy(payload, &rx_buf[5], payload_len);
    }

    return CAMERA_SPI_ERR_OK;
}

/* ===== payload 序列化/反序列化 ===== */

/**
 * @brief 序列化下行 payload 到元数据格式
 * @param payload 下行 payload 结构体
 * @param buffer 输出缓冲区（6+12=18 字节）
 */
static void camera_spi_serialize_downlink(const camera_spi_downlink_payload_t *payload,
                                          uint8 *buffer)
{
    uint16 length = payload->length;

    if(length > CAMERA_SPI_APP_DATA_CAPACITY)
    {
        length = CAMERA_SPI_APP_DATA_CAPACITY;
    }

    memset(buffer, 0, CAMERA_SPI_REQ_PAYLOAD_SIZE);
    camera_spi_write_u32_le(&buffer[0], payload->sequence);
    camera_spi_write_u16_le(&buffer[4], length);
    memcpy(&buffer[6], payload->data, CAMERA_SPI_APP_DATA_CAPACITY);
}

/**
 * @brief 反序列化上行 payload
 * @param buffer 上行元数据缓冲区（12+12=24 字节）
 * @param uplink 输出结构体
 * @return CAMERA_SPI_ERR_OK 或错误码
 */
static uint8 camera_spi_deserialize_uplink(const uint8 *buffer,
                                           camera_spi_uplink_payload_t *uplink)
{
    uint16 length;

    if((buffer == NULL) || (uplink == NULL))
    {
        return CAMERA_SPI_ERR_NULL_PTR;
    }

    length = camera_spi_read_u16_le(&buffer[8]);
    if(length > CAMERA_SPI_APP_DATA_CAPACITY)
    {
        return CAMERA_SPI_ERR_DATA_SIZE;
    }

    uplink->sequence = camera_spi_read_u32_le(&buffer[0]);
    uplink->ack_sequence = camera_spi_read_u32_le(&buffer[4]);
    uplink->length = length;
    uplink->flags = buffer[10];
    uplink->peer_last_error = buffer[11];
    memcpy(uplink->data, &buffer[12], CAMERA_SPI_APP_DATA_CAPACITY);

    return CAMERA_SPI_ERR_OK;
}

/**
 * @brief 构建同步请求帧（下行）
 * @param payload 下行 payload
 * @param buffer 输出帧缓冲区
 * @return 帧长度
 */
static uint16 camera_spi_build_sync_request(const camera_spi_downlink_payload_t *payload,
                                            uint8 *buffer)
{
    uint8 payload_buf[CAMERA_SPI_REQ_PAYLOAD_SIZE];

    camera_spi_serialize_downlink(payload, payload_buf);
    return camera_spi_build_frame(CAMERA_SPI_CMD_SYNC_DATA,
                                  payload_buf,
                                  CAMERA_SPI_REQ_PAYLOAD_SIZE,
                                  buffer,
                                  CAMERA_SPI_TRANSFER_LEN);
}

/**
 * @brief 解析同步响应帧（上行）
 * @param rx_buf 接收缓冲区
 * @param rx_len 接收长度
 * @param uplink 输出上行结构体
 * @return 错误码
 */
static uint8 camera_spi_parse_sync_response(const uint8 *rx_buf,
                                            uint16 rx_len,
                                            camera_spi_uplink_payload_t *uplink)
{
    uint8 payload[CAMERA_SPI_RESP_PAYLOAD_SIZE];
    uint8 ret;

    ret = camera_spi_parse_frame(rx_buf,
                                 rx_len,
                                 CAMERA_SPI_CMD_SYNC_DATA,
                                 payload,
                                 CAMERA_SPI_RESP_PAYLOAD_SIZE);
    if(ret != CAMERA_SPI_ERR_OK)
    {
        return ret;
    }

    return camera_spi_deserialize_uplink(payload, uplink);
}

/* ===== 调度 ===== */

/**
 * @brief round-robin 选择下一个要通信的从机
 *
 * 调度策略：从 ready_mask | downlink_mask 中按 round_robin_next 循环选取
 * 选中后推进 round_robin_next，保证公平轮转
 *
 * @param id 输出选中的从机 ID
 * @return 1=选中，0=无就绪从机
 */
static uint8 camera_spi_pick_next_slave(camera_spi_slave_id_t *id)
{
    uint8 index;
    uint8 candidate;
    uint8 schedule_mask = (uint8)(s_camera_spi_master.ready_mask |
                                  s_camera_spi_master.downlink_mask);

    if((id == NULL) || (schedule_mask == 0U))
    {
        return 0U;
    }

    for(index = 0U; index < CAMERA_SPI_SLAVE_COUNT; index++)
    {
        candidate = (uint8)((s_camera_spi_master.round_robin_next + index) %
                            CAMERA_SPI_SLAVE_COUNT);
        if((schedule_mask & camera_spi_slave_mask((camera_spi_slave_id_t)candidate)) != 0U)
        {
            *id = (camera_spi_slave_id_t)candidate;
            s_camera_spi_master.round_robin_next =
                (uint8)((candidate + 1U) % CAMERA_SPI_SLAVE_COUNT);
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief 刷新所有从机的 ready 引脚电平
 * 读取 GPIO 并更新 ready_mask（关中断保护）
 */
static void camera_spi_refresh_ready_levels(void)
{
    uint8 index;
    uint8 mask;
    uint32 lock;

    lock = interrupt_global_disable();
    for(index = 0U; index < CAMERA_SPI_SLAVE_COUNT; index++)
    {
        mask = camera_spi_slave_mask((camera_spi_slave_id_t)index);
        s_camera_spi_status[index].int_level =
            camera_spi_hw_get_ready_level((camera_spi_hw_slave_id_t)index);
        if(s_camera_spi_status[index].int_level != 0U)
        {
            s_camera_spi_master.ready_mask |= mask;
        }
    }
    interrupt_global_enable(lock);
}

/**
 * @brief 应用传输结果到从机状态
 *
 * 成功路径（ret==OK）：
 *   - online=1, ok_count++, 更新 uplink
 *   - 检查从机 ack_sequence 是否 >= 本次下行序号 → 更新同步状态
 *   - 如果从机已同步到最新 → 清除 downlink_mask
 *
 * 失败路径：
 *   - online=0, err_count++, 记录 last_error
 *   - 如果从机还没同步到最新 → 设置 downlink_mask（下次重传）
 *
 * @param id 从机 ID
 * @param ret 错误码
 * @param uplink 上行数据（失败时为 NULL）
 */
static void camera_spi_apply_result(camera_spi_slave_id_t id,
                                    uint8 ret,
                                    const camera_spi_uplink_payload_t *uplink)
{
    uint8 mask = camera_spi_slave_mask(id);

    s_camera_spi_status[id].int_level =
        camera_spi_hw_get_ready_level((camera_spi_hw_slave_id_t)id);

    if((ret == CAMERA_SPI_ERR_OK) && (uplink != NULL))
    {
        s_camera_spi_status[id].online = 1U;
        s_camera_spi_status[id].last_error = CAMERA_SPI_ERR_OK;
        s_camera_spi_status[id].ok_count++;
        s_camera_spi_status[id].last_update_ms = s_camera_spi_time_ms;
        s_camera_spi_status[id].uplink = *uplink;

        if(uplink->ack_sequence >= s_camera_spi_master.active_downlink_sequence)
        {
            s_camera_spi_downlink_synced[id] =
                s_camera_spi_master.active_downlink_sequence;
        }

        if(s_camera_spi_downlink_synced[id] >= s_camera_spi_downlink.sequence)
        {
            s_camera_spi_master.downlink_mask &= (uint8)(~mask);
        }
    }
    else
    {
        s_camera_spi_status[id].online = 0U;
        s_camera_spi_status[id].last_error = ret;
        s_camera_spi_status[id].err_count++;
        if(s_camera_spi_downlink_synced[id] < s_camera_spi_downlink.sequence)
        {
            s_camera_spi_master.downlink_mask |= mask;
        }
        else
        {
            s_camera_spi_master.downlink_mask &= (uint8)(~mask);
        }
    }
}

/**
 * @brief 发起下一个 SPI 传输
 *
 * 流程：
 *   1. 检查 transfer_busy（不重叠）
 *   2. pick_next_slave 选择从机
 *   3. 关中断清除 ready_mask
 *   4. 构建请求帧
 *   5. hw_start_transfer 启动硬件传输
 *
 * @return 1=传输已发起，0=无传输
 */
static uint8 camera_spi_start_next_transfer(void)
{
    camera_spi_slave_id_t id;
    uint8 ret;
    uint8 mask;
    uint32 lock;

    if(s_camera_spi_master.transfer_busy != 0U)
    {
        return 0U;
    }

    if(camera_spi_pick_next_slave(&id) == 0U)
    {
        return 0U;
    }

    /* 关中断清除该从机的 ready 标志 */
    mask = camera_spi_slave_mask(id);
    lock = interrupt_global_disable();
    s_camera_spi_master.ready_mask &= (uint8)(~mask);
    interrupt_global_enable(lock);

    /* 构建请求帧 */
    if(camera_spi_build_sync_request(&s_camera_spi_downlink,
                                     s_camera_spi_master.tx_buf) != CAMERA_SPI_REQ_FRAME_LEN)
    {
        camera_spi_apply_result(id, CAMERA_SPI_ERR_DATA_SIZE, NULL);
        return 0U;
    }

    /* 准备接收缓冲区 */
    memset(s_camera_spi_master.rx_buf, 0xFF, sizeof(s_camera_spi_master.rx_buf));
    s_camera_spi_master.active_slave = (uint8)id;
    s_camera_spi_master.active_downlink_sequence = s_camera_spi_downlink.sequence;
    s_camera_spi_master.transfer_poll_guard = 0U;

    /* 启动硬件传输 */
    ret = camera_spi_hw_start_transfer((camera_spi_hw_slave_id_t)id,
                                       s_camera_spi_master.tx_buf,
                                       s_camera_spi_master.rx_buf,
                                       CAMERA_SPI_TRANSFER_LEN);
    if(ret == CAMERA_SPI_HW_TRANSFER_OK)
    {
        s_camera_spi_master.transfer_busy = 1U;
        s_camera_spi_diag.transfer_busy = 1U;
        s_camera_spi_diag.active_slave = (uint8)id;
        s_camera_spi_diag.transfer_start_count++;
        return 1U;
    }

    /* 硬件启动失败 */
    camera_spi_apply_result(id, CAMERA_SPI_ERR_HW, NULL);
    s_camera_spi_diag.transfer_error_count++;
    return 0U;
}

/**
 * @brief 处理完成的 SPI 传输
 *
 * 两种触发方式：
 *   - forced_timeout=0: hw_transfer_finished() 返回完成 → 正常完成
 *   - forced_timeout=1: 轮询超时 → abort 后以超时错误处理
 *
 * 正常完成：解析响应帧 → 更新从机状态
 * 超时/错误：调 abort（如需）→ 记录错误 → 更新从机状态
 *
 * @param forced_timeout 1=强制超时
 */
static void camera_spi_finish_active_transfer(uint8 forced_timeout)
{
    camera_spi_slave_id_t id = (camera_spi_slave_id_t)s_camera_spi_master.active_slave;
    camera_spi_uplink_payload_t uplink;
    uint8 ret;

    if(id >= CAMERA_SPI_SLAVE_COUNT)
    {
        s_camera_spi_master.transfer_busy = 0U;
        return;
    }

    if(forced_timeout != 0U)
    {
        camera_spi_hw_abort_transfer();
        ret = CAMERA_SPI_ERR_TIMEOUT;
        s_camera_spi_diag.timeout_count++;
    }
    else
    {
        ret = camera_spi_hw_finish_transfer();
        if(ret == CAMERA_SPI_HW_TRANSFER_BUSY)
        {
            /* 还没完成，等下次 poll */
            return;
        }
        if(ret != CAMERA_SPI_HW_TRANSFER_OK)
        {
            ret = CAMERA_SPI_ERR_HW;
        }
        else
        {
            /* 解析响应帧 */
            ret = camera_spi_parse_sync_response(s_camera_spi_master.rx_buf,
                                                 CAMERA_SPI_TRANSFER_LEN,
                                                 &uplink);
        }
    }

    s_camera_spi_master.transfer_busy = 0U;
    s_camera_spi_diag.transfer_busy = 0U;
    s_camera_spi_diag.active_slave = 0U;

    if(ret == CAMERA_SPI_ERR_OK)
    {
        s_camera_spi_diag.transfer_done_count++;
        camera_spi_apply_result(id, ret, &uplink);
    }
    else
    {
        s_camera_spi_diag.transfer_error_count++;
        s_camera_spi_diag.last_error = ret;
        camera_spi_apply_result(id, ret, NULL);
    }
}

/* ===== 目标解析 ===== */

/**
 * @brief 从从机状态解码目标检测结果
 *
 * 上行 data 布局（12 字节 = 3 个 uint32 LE）：
 *   word0[0:1] = frame_id, word0[2:3] = spot_count
 *   word1[0:1] = spot_index, word1[2:3] = x (int16)
 *   word2[0:1] = y (int16), word2[2:3] = area
 *
 * @param id 从机 ID
 * @param target 输出目标结构体
 * @return 1=有效目标，0=无有效目标
 */
static uint8 camera_spi_decode_target_from_status(camera_spi_slave_id_t id,
                                                  camera_spi_target_t *target)
{
    const camera_spi_slave_status_t *status;
    uint32 word0;
    uint32 word1;
    uint32 word2;
    uint16 frame_id;
    uint16 spot_count;
    uint16 spot_index;
    int16 x;
    int16 y;
    uint16 area;

    if((id >= CAMERA_SPI_SLAVE_COUNT) || (target == NULL))
    {
        return 0U;
    }

    status = &s_camera_spi_status[id];
    if((status->online == 0U) || (status->uplink.length < CAMERA_SPI_APP_DATA_CAPACITY))
    {
        return 0U;
    }

    word0 = camera_spi_read_u32_le(&status->uplink.data[0]);
    word1 = camera_spi_read_u32_le(&status->uplink.data[4]);
    word2 = camera_spi_read_u32_le(&status->uplink.data[8]);

    frame_id = (uint16)(word0 & 0xFFFFU);
    spot_count = (uint16)((word0 >> 16) & 0xFFFFU);
    spot_index = (uint16)(word1 & 0xFFFFU);
    x = (int16)((word1 >> 16) & 0xFFFFU);
    y = (int16)(word2 & 0xFFFFU);
    area = (uint16)((word2 >> 16) & 0xFFFFU);

    /* 有效性检查 */
    if((spot_count == 0U) || (spot_index >= spot_count) || (x < 0) || (y < 0))
    {
        return 0U;
    }

    memset(target, 0, sizeof(*target));
    target->valid = 1U;
    target->camera_id = (uint8)id;
    target->frame_id = frame_id;
    target->spot_count = spot_count;
    target->spot_index = spot_index;
    target->x = (uint16)x;
    target->y = (uint16)y;
    target->area = area;
    target->uplink_sequence = status->uplink.sequence;
    target->last_update_ms = status->last_update_ms;
    target->age_ms = s_camera_spi_time_ms - status->last_update_ms;

    return 1U;
}

/**
 * @brief 刷新最佳目标：从三路摄像头中选 area 最大的有效目标
 */
static void camera_spi_refresh_target(void)
{
    camera_spi_target_t candidate;
    camera_spi_target_t best;
    uint8 index;

    memset(&best, 0, sizeof(best));

    for(index = 0U; index < CAMERA_SPI_SLAVE_COUNT; index++)
    {
        if(camera_spi_decode_target_from_status((camera_spi_slave_id_t)index, &candidate) != 0U)
        {
            if((best.valid == 0U) || (candidate.area > best.area))
            {
                best = candidate;
            }
        }
    }

    s_camera_spi_target = best;
}

/* ===== 公共接口 ===== */

void camera_spi_init(void)
{
    memset(&s_camera_spi_master, 0, sizeof(s_camera_spi_master));
    memset(s_camera_spi_status, 0, sizeof(s_camera_spi_status));
    memset(&s_camera_spi_downlink, 0, sizeof(s_camera_spi_downlink));
    memset(s_camera_spi_downlink_synced, 0, sizeof(s_camera_spi_downlink_synced));
    memset(&s_camera_spi_diag, 0, sizeof(s_camera_spi_diag));
    memset(&s_camera_spi_target, 0, sizeof(s_camera_spi_target));
    s_camera_spi_time_ms = 0U;
    s_camera_spi_master.active_slave = CAMERA_SPI_SLAVE_COUNT;
    camera_spi_hw_init();
}

/**
 * @brief 主循环轮询入口
 * 调用频率：越高越好
 *
 * 优先级：
 *   1. 有传输在进行 → 检查完成/超时
 *   2. 无传输 → 刷新 ready → 发起新传输
 */
void camera_spi_poll(void)
{
    s_camera_spi_diag.poll_count++;

    if(s_camera_spi_master.transfer_busy != 0U)
    {
        s_camera_spi_master.transfer_poll_guard++;
        if(camera_spi_hw_transfer_finished() != 0U)
        {
            camera_spi_finish_active_transfer(0U);
        }
        else if(s_camera_spi_master.transfer_poll_guard > CAMERA_SPI_POLL_TIMEOUT_COUNT)
        {
            /* 软件超时：强制终止传输 */
            camera_spi_finish_active_transfer(1U);
        }
        return;
    }

    camera_spi_refresh_ready_levels();
    (void)camera_spi_start_next_transfer();
}

void camera_spi_update_100HZ(uint32 system_time_ms)
{
    s_camera_spi_time_ms = system_time_ms;
    camera_spi_refresh_ready_levels();
    camera_spi_refresh_target();
}

void camera_spi_notify_ready(camera_spi_slave_id_t id)
{
    if(id >= CAMERA_SPI_SLAVE_COUNT)
    {
        return;
    }

    s_camera_spi_master.ready_mask |= camera_spi_slave_mask(id);
    s_camera_spi_status[id].int_level = 1U;
    s_camera_spi_diag.ready_irq_count[id]++;
}

uint8 camera_spi_get_status(camera_spi_slave_id_t id, camera_spi_slave_status_t *status)
{
    if((id >= CAMERA_SPI_SLAVE_COUNT) || (status == NULL))
    {
        return CAMERA_SPI_ERR_INVALID_SLAVE;
    }

    *status = s_camera_spi_status[id];
    return CAMERA_SPI_ERR_OK;
}

uint8 camera_spi_get_diag(camera_spi_diag_t *diag)
{
    if(diag == NULL)
    {
        return CAMERA_SPI_ERR_NULL_PTR;
    }

    *diag = s_camera_spi_diag;
    diag->transfer_busy = s_camera_spi_master.transfer_busy;
    diag->active_slave = s_camera_spi_master.active_slave;
    diag->ready_mask = s_camera_spi_master.ready_mask;
    diag->downlink_mask = s_camera_spi_master.downlink_mask;
    return CAMERA_SPI_ERR_OK;
}

uint8 camera_spi_get_target(camera_spi_target_t *target)
{
    if(target == NULL)
    {
        return CAMERA_SPI_ERR_NULL_PTR;
    }

    *target = s_camera_spi_target;
    return s_camera_spi_target.valid;
}

void camera_spi_set_downlink_payload(const camera_spi_payload_buffer_t *payload)
{
    uint16 length;

    if(payload == NULL)
    {
        return;
    }

    length = payload->length;
    if(length > CAMERA_SPI_APP_DATA_CAPACITY)
    {
        length = CAMERA_SPI_APP_DATA_CAPACITY;
    }

    s_camera_spi_downlink.sequence++;
    s_camera_spi_downlink.length = length;
    memset(s_camera_spi_downlink.data, 0, sizeof(s_camera_spi_downlink.data));
    if(length > 0U)
    {
        memcpy(s_camera_spi_downlink.data, payload->data, length);
    }
    /* 标记所有从机需要接收新数据 */
    s_camera_spi_master.downlink_mask = CAMERA_SPI_ALL_SLAVE_MASK;
}
