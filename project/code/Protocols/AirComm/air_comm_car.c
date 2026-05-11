#include "air_comm_car.h"

#define AIR_COMM_HEADER_0                  (0xAAU)
#define AIR_COMM_HEADER_1                  (0xAAU)
#define AIR_COMM_HEADER_2                  (0x55U)
#define AIR_COMM_HEADER_3                  (0x55U)

#define AIR_COMM_MAX_PAYLOAD               (250U)
#define AIR_COMM_FRAME_OVERHEAD            (9U)
#define AIR_COMM_MAX_FRAME                 (AIR_COMM_MAX_PAYLOAD + AIR_COMM_FRAME_OVERHEAD)
#define AIR_COMM_RX_QUEUE_SIZE             (512U)

#define AIR_COMM_MSG_SET_PARAM             (0x01U)
#define AIR_COMM_MSG_ACK_PARAM             (0x02U)
#define AIR_COMM_MSG_EXEC_FUNC             (0x03U)
#define AIR_COMM_MSG_ACK_FUNC              (0x04U)
#define AIR_COMM_MSG_HEARTBEAT             (0x05U)
#define AIR_COMM_MSG_RUN_DATA              (0x06U)

#define COMM_ACK_TIMEOUT_MS                (200U)
#define COMM_MAX_RETRY                     (3U)
#define COMM_HEARTBEAT_MS                  (200U)
#define COMM_OFFLINE_MS                    (600U)
#define AIR_COMM_TOTAL_ACK_TIMEOUT_MS      ((COMM_MAX_RETRY + 1U) * COMM_ACK_TIMEOUT_MS + 50U)

#define AIR_COMM_ACK_RESULT_NONE           (0U)
#define AIR_COMM_ACK_RESULT_OK             (1U)
#define AIR_COMM_ACK_RESULT_TIMEOUT        (2U)
#define AIR_COMM_ACK_RESULT_ERROR          (3U)

typedef struct
{
    uint8 state;
    uint8 header_count;
    uint8 type;
    uint8 seq;
    uint8 len;
    uint8 payload[AIR_COMM_MAX_PAYLOAD];
    uint16 payload_count;
    uint16 crc;
    uint8 crc_count;
} air_comm_rx_state_t;

typedef struct
{
    uint8 active;
    uint8 type;
    uint8 seq;
    uint8 retry;
    uint8 result;
    uint8 status;
    uint32 send_time;
    uint8 frame[AIR_COMM_MAX_FRAME];
    uint16 frame_len;
} air_comm_ack_state_t;

typedef struct
{
    uint8 data[AIR_COMM_RX_QUEUE_SIZE];
    volatile uint16 head;
    volatile uint16 tail;
} air_comm_rx_queue_t;

static air_comm_rx_state_t s_air_comm_rx;
static air_comm_ack_state_t s_air_comm_ack;
static air_comm_rx_queue_t s_air_comm_rx_queue;
static air_comm_stats_t s_air_comm_stats;
static air_comm_run_data_fn s_air_comm_run_data_callback;
static volatile uint32 s_air_comm_tick_ms;
static uint32 s_air_comm_last_peer_ms;
static uint32 s_air_comm_last_heartbeat_ms;
static uint8 s_air_comm_seq;
static uint8 s_air_comm_initialized;

static uint16 air_comm_crc16(const uint8 *data, uint16 len)
{
    uint16 crc = 0xFFFFU;
    uint16 i;
    uint8 j;

    for(i = 0U; i < len; i++)
    {
        crc ^= (uint16)data[i] << 8;
        for(j = 0U; j < 8U; j++)
        {
            if((crc & 0x8000U) != 0U)
            {
                crc = (uint16)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

static float air_comm_read_float(const uint8 *buffer)
{
    float value;
    uint8 *ptr = (uint8 *)&value;

    ptr[0] = buffer[0];
    ptr[1] = buffer[1];
    ptr[2] = buffer[2];
    ptr[3] = buffer[3];
    return value;
}

static void air_comm_write_float(uint8 *buffer, float value)
{
    uint8 *ptr = (uint8 *)&value;

    buffer[0] = ptr[0];
    buffer[1] = ptr[1];
    buffer[2] = ptr[2];
    buffer[3] = ptr[3];
}

static void air_comm_write_u32(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

static uint8 air_comm_send_uart(const uint8 *data, uint16 len)
{
    if((data == NULL) || (len == 0U))
    {
        return 0U;
    }

    uart_write_buffer(UART_3, data, len);
    return 1U;
}

static uint8 air_comm_expected_ack_type(uint8 request_type)
{
    if(request_type == AIR_COMM_MSG_SET_PARAM)
    {
        return AIR_COMM_MSG_ACK_PARAM;
    }
    if(request_type == AIR_COMM_MSG_EXEC_FUNC)
    {
        return AIR_COMM_MSG_ACK_FUNC;
    }
    return 0U;
}

static uint8 air_comm_parse_ack_status(uint8 ack_type,
                                       const uint8 *payload,
                                       uint8 len)
{
    if(payload == NULL)
    {
        return AIR_COMM_STATUS_ERROR;
    }

    if(ack_type == AIR_COMM_MSG_ACK_PARAM)
    {
        return (len >= 2U) ? payload[0] : AIR_COMM_STATUS_ERROR;
    }

    if(ack_type == AIR_COMM_MSG_ACK_FUNC)
    {
        return (len >= 6U) ? payload[1] : AIR_COMM_STATUS_ERROR;
    }

    return AIR_COMM_STATUS_ERROR;
}

static uint8 air_comm_send_frame(uint8 type,
                                 uint8 seq,
                                 const uint8 *payload,
                                 uint8 len,
                                 uint8 save_for_ack)
{
    uint8 frame[AIR_COMM_MAX_FRAME];
    uint16 pos = 0U;
    uint16 crc;

    if((len > AIR_COMM_MAX_PAYLOAD) || ((len > 0U) && (payload == NULL)))
    {
        return 0U;
    }

    frame[pos++] = AIR_COMM_HEADER_0;
    frame[pos++] = AIR_COMM_HEADER_1;
    frame[pos++] = AIR_COMM_HEADER_2;
    frame[pos++] = AIR_COMM_HEADER_3;
    frame[pos++] = type;
    frame[pos++] = seq;
    frame[pos++] = len;

    if((len > 0U) && (payload != NULL))
    {
        memcpy(&frame[pos], payload, len);
        pos += len;
    }

    crc = air_comm_crc16(frame, pos);
    frame[pos++] = (uint8)(crc & 0xFFU);
    frame[pos++] = (uint8)((crc >> 8) & 0xFFU);

    if(air_comm_send_uart(frame, pos) == 0U)
    {
        return 0U;
    }

    if(save_for_ack != 0U)
    {
        memcpy(s_air_comm_ack.frame, frame, pos);
        s_air_comm_ack.frame_len = pos;
    }

    s_air_comm_stats.tx_frame_count++;
    s_air_comm_stats.tx_byte_count += pos;
    if(type == AIR_COMM_MSG_HEARTBEAT)
    {
        s_air_comm_stats.heartbeat_tx_count++;
    }

    return 1U;
}

static uint8 air_comm_send(uint8 type, const uint8 *payload, uint8 len, uint8 need_ack)
{
    uint8 seq;

    if((need_ack != 0U) && (s_air_comm_ack.active != 0U))
    {
        return 0U;
    }

    seq = s_air_comm_seq;
    if(air_comm_send_frame(type, seq, payload, len, need_ack) == 0U)
    {
        return 0U;
    }

    s_air_comm_seq++;
    if(need_ack != 0U)
    {
        s_air_comm_ack.active = 1U;
        s_air_comm_ack.type = type;
        s_air_comm_ack.seq = seq;
        s_air_comm_ack.send_time = s_air_comm_tick_ms;
        s_air_comm_ack.retry = 0U;
        s_air_comm_ack.result = AIR_COMM_ACK_RESULT_NONE;
        s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
    }

    return 1U;
}

static void air_comm_send_heartbeat(void)
{
    uint8 payload[6];

    payload[0] = 0U;
    payload[1] = 0U;
    air_comm_write_u32(&payload[2], s_air_comm_tick_ms);
    if(air_comm_send_frame(AIR_COMM_MSG_HEARTBEAT, s_air_comm_seq, payload, 6U, 0U) != 0U)
    {
        s_air_comm_seq++;
    }
}

static void air_comm_mark_peer_heartbeat(void)
{
    s_air_comm_last_peer_ms = s_air_comm_tick_ms;
    s_air_comm_stats.online_status = 1U;
}

static void air_comm_match_ack(uint8 ack_type,
                               uint8 seq,
                               const uint8 *payload,
                               uint8 len)
{
    uint8 expected;
    uint8 status;

    if(s_air_comm_ack.active == 0U)
    {
        return;
    }

    expected = air_comm_expected_ack_type(s_air_comm_ack.type);
    if((ack_type != expected) || (seq != s_air_comm_ack.seq))
    {
        return;
    }

    status = air_comm_parse_ack_status(ack_type, payload, len);

    s_air_comm_ack.active = 0U;
    s_air_comm_ack.status = status;
    s_air_comm_stats.last_ack_status = status;
    if(status == AIR_COMM_STATUS_OK)
    {
        s_air_comm_ack.result = AIR_COMM_ACK_RESULT_OK;
        s_air_comm_stats.ack_ok_count++;
    }
    else
    {
        s_air_comm_ack.result = AIR_COMM_ACK_RESULT_ERROR;
    }
}

static void air_comm_handle_run_data(const uint8 *payload, uint8 len)
{
    uint8 count;
    uint16 required_len;
    uint8 index;
    float data[AIR_COMM_RUN_DATA_MAX_FLOATS];

    if((payload == NULL) || (len < 1U))
    {
        return;
    }

    count = payload[0];
    if(count > AIR_COMM_RUN_DATA_MAX_FLOATS)
    {
        return;
    }

    required_len = (uint16)(1U + ((uint16)count * 4U));
    if((uint16)len < required_len)
    {
        return;
    }

    for(index = 0U; index < count; index++)
    {
        data[index] = air_comm_read_float(&payload[1U + ((uint16)index * 4U)]);
    }

    if(s_air_comm_run_data_callback != NULL)
    {
        s_air_comm_run_data_callback(data, count);
    }
}

static void air_comm_handle_frame(uint8 type,
                                  uint8 seq,
                                  const uint8 *payload,
                                  uint8 len)
{
    switch(type)
    {
        case AIR_COMM_MSG_ACK_PARAM:
        case AIR_COMM_MSG_ACK_FUNC:
            air_comm_match_ack(type, seq, payload, len);
            break;

        case AIR_COMM_MSG_HEARTBEAT:
            s_air_comm_stats.heartbeat_rx_count++;
            air_comm_mark_peer_heartbeat();
            break;

        case AIR_COMM_MSG_RUN_DATA:
            air_comm_handle_run_data(payload, len);
            break;

        default:
            break;
    }
}

static void air_comm_process_rx_frame(void)
{
    uint8 frame[AIR_COMM_MAX_FRAME];
    uint16 pos = 0U;
    uint16 crc_calc;

    frame[pos++] = AIR_COMM_HEADER_0;
    frame[pos++] = AIR_COMM_HEADER_1;
    frame[pos++] = AIR_COMM_HEADER_2;
    frame[pos++] = AIR_COMM_HEADER_3;
    frame[pos++] = s_air_comm_rx.type;
    frame[pos++] = s_air_comm_rx.seq;
    frame[pos++] = s_air_comm_rx.len;

    if(s_air_comm_rx.len > 0U)
    {
        memcpy(&frame[pos], s_air_comm_rx.payload, s_air_comm_rx.len);
        pos += s_air_comm_rx.len;
    }

    crc_calc = air_comm_crc16(frame, pos);
    if(crc_calc != s_air_comm_rx.crc)
    {
        s_air_comm_stats.crc_error_count++;
        return;
    }

    s_air_comm_stats.rx_frame_count++;
    s_air_comm_stats.rx_byte_count += (uint32)(pos + 2U);
    air_comm_handle_frame(s_air_comm_rx.type,
                          s_air_comm_rx.seq,
                          s_air_comm_rx.payload,
                          s_air_comm_rx.len);
}

static void air_comm_rx_byte_parser(uint8 byte)
{
    s_air_comm_stats.rx_raw_byte_count++;

    switch(s_air_comm_rx.state)
    {
        case 0:
            if((s_air_comm_rx.header_count == 0U) && (byte == AIR_COMM_HEADER_0))
            {
                s_air_comm_rx.header_count = 1U;
            }
            else if((s_air_comm_rx.header_count == 1U) && (byte == AIR_COMM_HEADER_1))
            {
                s_air_comm_rx.header_count = 2U;
            }
            else if((s_air_comm_rx.header_count == 2U) && (byte == AIR_COMM_HEADER_2))
            {
                s_air_comm_rx.header_count = 3U;
            }
            else if((s_air_comm_rx.header_count == 3U) && (byte == AIR_COMM_HEADER_3))
            {
                s_air_comm_rx.header_count = 0U;
                s_air_comm_rx.state = 1U;
            }
            else
            {
                s_air_comm_rx.header_count = (byte == AIR_COMM_HEADER_0) ? 1U : 0U;
            }
            break;

        case 1:
            if(s_air_comm_rx.header_count == 0U)
            {
                s_air_comm_rx.type = byte;
                s_air_comm_rx.header_count = 1U;
            }
            else if(s_air_comm_rx.header_count == 1U)
            {
                s_air_comm_rx.seq = byte;
                s_air_comm_rx.header_count = 2U;
            }
            else
            {
                s_air_comm_rx.len = byte;
                s_air_comm_rx.header_count = 0U;
                s_air_comm_rx.payload_count = 0U;
                if(byte > AIR_COMM_MAX_PAYLOAD)
                {
                    s_air_comm_stats.rx_oversize_count++;
                    s_air_comm_rx.state = 0U;
                }
                else if(byte == 0U)
                {
                    s_air_comm_rx.state = 3U;
                    s_air_comm_rx.crc_count = 0U;
                }
                else
                {
                    s_air_comm_rx.state = 2U;
                }
            }
            break;

        case 2:
            if(s_air_comm_rx.payload_count < AIR_COMM_MAX_PAYLOAD)
            {
                s_air_comm_rx.payload[s_air_comm_rx.payload_count++] = byte;
            }
            if(s_air_comm_rx.payload_count >= s_air_comm_rx.len)
            {
                s_air_comm_rx.state = 3U;
                s_air_comm_rx.crc_count = 0U;
            }
            break;

        case 3:
            if(s_air_comm_rx.crc_count == 0U)
            {
                s_air_comm_rx.crc = byte;
                s_air_comm_rx.crc_count = 1U;
            }
            else
            {
                s_air_comm_rx.crc |= (uint16)byte << 8;
                air_comm_process_rx_frame();
                s_air_comm_rx.state = 0U;
                s_air_comm_rx.header_count = 0U;
            }
            break;

        default:
            s_air_comm_rx.state = 0U;
            s_air_comm_rx.header_count = 0U;
            break;
    }
}

static uint8 air_comm_rx_queue_pop(uint8 *byte)
{
    uint16 tail;

    if(byte == NULL)
    {
        return 0U;
    }

    if(s_air_comm_rx_queue.head == s_air_comm_rx_queue.tail)
    {
        return 0U;
    }

    tail = s_air_comm_rx_queue.tail;
    *byte = s_air_comm_rx_queue.data[tail];
    tail++;
    if(tail >= AIR_COMM_RX_QUEUE_SIZE)
    {
        tail = 0U;
    }
    s_air_comm_rx_queue.tail = tail;
    return 1U;
}

static void air_comm_task_ack_and_online(void)
{
    if(s_air_comm_ack.active != 0U)
    {
        if((s_air_comm_tick_ms - s_air_comm_ack.send_time) >= COMM_ACK_TIMEOUT_MS)
        {
            if(s_air_comm_ack.retry < COMM_MAX_RETRY)
            {
                if(air_comm_send_uart(s_air_comm_ack.frame, s_air_comm_ack.frame_len) != 0U)
                {
                    s_air_comm_ack.send_time = s_air_comm_tick_ms;
                    s_air_comm_ack.retry++;
                    s_air_comm_stats.ack_retry_count++;
                    s_air_comm_stats.tx_frame_count++;
                    s_air_comm_stats.tx_byte_count += s_air_comm_ack.frame_len;
                }
            }
            else
            {
                s_air_comm_ack.active = 0U;
                s_air_comm_ack.result = AIR_COMM_ACK_RESULT_TIMEOUT;
                s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
                s_air_comm_stats.last_ack_status = AIR_COMM_STATUS_ERROR;
                s_air_comm_stats.ack_timeout_count++;
            }
        }
    }

    if(s_air_comm_last_peer_ms != 0U)
    {
        if((s_air_comm_tick_ms - s_air_comm_last_peer_ms) >= COMM_OFFLINE_MS)
        {
            s_air_comm_stats.online_status = 2U;
            s_air_comm_last_peer_ms = 0U;
        }
    }
}

void air_comm_car_init(void)
{
    memset(&s_air_comm_rx, 0, sizeof(s_air_comm_rx));
    memset(&s_air_comm_ack, 0, sizeof(s_air_comm_ack));
    memset(&s_air_comm_rx_queue, 0, sizeof(s_air_comm_rx_queue));
    memset(&s_air_comm_stats, 0, sizeof(s_air_comm_stats));
    s_air_comm_run_data_callback = NULL;
    s_air_comm_tick_ms = 0U;
    s_air_comm_last_peer_ms = 0U;
    s_air_comm_last_heartbeat_ms = 0U;
    s_air_comm_seq = 0U;
    s_air_comm_initialized = 1U;

    uart_init(UART_3, AIR_COMM_BAUDRATE, UART3_TX_P17_2, UART3_RX_P17_1);
    uart_rx_interrupt(UART_3, 1U);
}

void air_comm_car_tick_1MS(void)
{
    s_air_comm_tick_ms++;
}

void air_comm_car_poll(void)
{
    uint8 byte;
    uint16 guard = AIR_COMM_RX_QUEUE_SIZE;

    if(s_air_comm_initialized == 0U)
    {
        return;
    }

    while((guard > 0U) && (air_comm_rx_queue_pop(&byte) != 0U))
    {
        air_comm_rx_byte_parser(byte);
        guard--;
    }

    air_comm_task_ack_and_online();
}

void air_comm_car_update_100HZ(void)
{
    if(s_air_comm_initialized == 0U)
    {
        return;
    }

    if((s_air_comm_tick_ms - s_air_comm_last_heartbeat_ms) >= COMM_HEARTBEAT_MS)
    {
        air_comm_send_heartbeat();
        s_air_comm_last_heartbeat_ms = s_air_comm_tick_ms;
    }

    air_comm_task_ack_and_online();
}

void air_comm_car_rx_byte(uint8 byte)
{
    uint16 next_head;

    next_head = s_air_comm_rx_queue.head + 1U;
    if(next_head >= AIR_COMM_RX_QUEUE_SIZE)
    {
        next_head = 0U;
    }

    if(next_head == s_air_comm_rx_queue.tail)
    {
        s_air_comm_stats.rx_queue_overflow_count++;
        return;
    }

    s_air_comm_rx_queue.data[s_air_comm_rx_queue.head] = byte;
    s_air_comm_rx_queue.head = next_head;
}

uint8 air_comm_car_is_online(void)
{
    return (s_air_comm_stats.online_status == 1U) ? 1U : 0U;
}

uint8 air_comm_car_get_online_status(void)
{
    return s_air_comm_stats.online_status;
}

uint32 air_comm_car_get_tick(void)
{
    return s_air_comm_tick_ms;
}

uint8 air_comm_car_set_param(const char *name, float value)
{
    uint8 payload[1U + AIR_COMM_PARAM_NAME_MAX + 4U];
    uint16 name_len;
    uint8 pos = 0U;
    uint32 start_ms;

    if((name == NULL) || (s_air_comm_initialized == 0U))
    {
        return 1U;
    }

    name_len = (uint16)strlen(name);
    if((name_len == 0U) || (name_len > AIR_COMM_PARAM_NAME_MAX))
    {
        return 1U;
    }

    payload[pos++] = (uint8)name_len;
    memcpy(&payload[pos], name, name_len);
    pos = (uint8)(pos + (uint8)name_len);
    air_comm_write_float(&payload[pos], value);
    pos = (uint8)(pos + 4U);

    if(air_comm_send(AIR_COMM_MSG_SET_PARAM, payload, pos, 1U) == 0U)
    {
        return 1U;
    }

    start_ms = s_air_comm_tick_ms;
    while(s_air_comm_ack.active != 0U)
    {
        air_comm_car_poll();
        if((s_air_comm_tick_ms - start_ms) >= AIR_COMM_TOTAL_ACK_TIMEOUT_MS)
        {
            s_air_comm_ack.active = 0U;
            s_air_comm_ack.result = AIR_COMM_ACK_RESULT_TIMEOUT;
            s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
            s_air_comm_stats.last_ack_status = AIR_COMM_STATUS_ERROR;
            return 1U;
        }
    }

    return ((s_air_comm_ack.result == AIR_COMM_ACK_RESULT_OK) &&
            (s_air_comm_ack.status == AIR_COMM_STATUS_OK)) ? 0U : 1U;
}

uint8 air_comm_car_exec_func(uint8 func_id)
{
    uint8 payload[2];
    uint32 start_ms;

    if(s_air_comm_initialized == 0U)
    {
        return 1U;
    }

    payload[0] = func_id;
    payload[1] = 0U;

    if(air_comm_send(AIR_COMM_MSG_EXEC_FUNC, payload, 2U, 1U) == 0U)
    {
        return 1U;
    }

    start_ms = s_air_comm_tick_ms;
    while(s_air_comm_ack.active != 0U)
    {
        air_comm_car_poll();
        if((s_air_comm_tick_ms - start_ms) >= AIR_COMM_TOTAL_ACK_TIMEOUT_MS)
        {
            s_air_comm_ack.active = 0U;
            s_air_comm_ack.result = AIR_COMM_ACK_RESULT_TIMEOUT;
            s_air_comm_ack.status = AIR_COMM_STATUS_ERROR;
            s_air_comm_stats.last_ack_status = AIR_COMM_STATUS_ERROR;
            return 1U;
        }
    }

    return ((s_air_comm_ack.result == AIR_COMM_ACK_RESULT_OK) &&
            (s_air_comm_ack.status == AIR_COMM_STATUS_OK)) ? 0U : 1U;
}

void air_comm_car_set_run_data_callback(air_comm_run_data_fn callback)
{
    s_air_comm_run_data_callback = callback;
}

void air_comm_car_get_stats(air_comm_stats_t *stats)
{
    if(stats == NULL)
    {
        return;
    }

    s_air_comm_stats.tick_ms = s_air_comm_tick_ms;
    s_air_comm_stats.pending_ack = s_air_comm_ack.active;
    *stats = s_air_comm_stats;
}
