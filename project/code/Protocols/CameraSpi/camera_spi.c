/*
 * Camera SPI master.
 *
 * Public API is raw only. The module keeps the fixed SPI frame format inside:
 * AA 55 CMD LEN_BE PAYLOAD CRC16_LE ED.
 */

#include "camera_spi.h"
#include "HW_Drivers/CameraSpi/camera_spi_hw.h"

#define CAMERA_SPI_FRAME_HEAD_1       (0xAAU)
#define CAMERA_SPI_FRAME_HEAD_2       (0x55U)
#define CAMERA_SPI_FRAME_TAIL         (0xEDU)
#define CAMERA_SPI_CMD_SYNC_DATA      (0x20U)

#define CAMERA_SPI_FRAME_OVERHEAD     (8U)
#define CAMERA_SPI_MAX_DATA_SIZE      (100U)
#define CAMERA_SPI_REQ_META_SIZE      (6U)
#define CAMERA_SPI_RESP_META_SIZE     (12U)
#define CAMERA_SPI_REQ_PAYLOAD_SIZE   (CAMERA_SPI_REQ_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)
#define CAMERA_SPI_RESP_PAYLOAD_SIZE  (CAMERA_SPI_RESP_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)
#define CAMERA_SPI_REQ_FRAME_LEN      (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_REQ_PAYLOAD_SIZE)
#define CAMERA_SPI_RESP_FRAME_LEN     (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_RESP_PAYLOAD_SIZE)
#define CAMERA_SPI_TRANSFER_LEN       CAMERA_SPI_RESP_FRAME_LEN
#define CAMERA_SPI_POLL_TIMEOUT_COUNT (50000U)

#define CAMERA_SPI_ERR_OK             (0U)
#define CAMERA_SPI_ERR_FRAME_SHORT    (1U)
#define CAMERA_SPI_ERR_INVALID_HEAD   (2U)
#define CAMERA_SPI_ERR_PAYLOAD_LONG   (3U)
#define CAMERA_SPI_ERR_INVALID_TAIL   (4U)
#define CAMERA_SPI_ERR_CRC            (5U)
#define CAMERA_SPI_ERR_INCOMPLETE     (6U)
#define CAMERA_SPI_ERR_NULL_PTR       (7U)
#define CAMERA_SPI_ERR_TIMEOUT        (8U)
#define CAMERA_SPI_ERR_DATA_SIZE      (9U)
#define CAMERA_SPI_ERR_INVALID_SLAVE  (10U)
#define CAMERA_SPI_ERR_INVALID_CMD    (11U)
#define CAMERA_SPI_ERR_HW             (12U)

typedef struct
{
    uint32 sequence;
    uint16 length;
    uint8 data[CAMERA_SPI_APP_DATA_CAPACITY];
} camera_spi_downlink_payload_t;

typedef struct
{
    uint32 sequence;
    uint32 ack_sequence;
    uint16 length;
    uint8 flags;
    uint8 peer_last_error;
    uint8 data[CAMERA_SPI_APP_DATA_CAPACITY];
} camera_spi_uplink_payload_t;

typedef struct
{
    uint8 online;
    uint8 ready_level;
    uint8 last_error;
    uint8 last_rx_head0;
    uint8 last_rx_head1;
    uint32 ok_count;
    uint32 err_count;
    camera_spi_uplink_payload_t uplink;
} camera_spi_slave_status_t;

typedef struct
{
    uint8 transfer_busy;
    uint8 active_slave;
    uint8 round_robin_next;
    uint8 ready_mask;
    uint8 downlink_mask;
    uint32 transfer_poll_guard;
    uint8 tx_buf[CAMERA_SPI_TRANSFER_LEN];
    uint8 rx_buf[CAMERA_SPI_TRANSFER_LEN];
} camera_spi_master_state_t;

static camera_spi_master_state_t s_master;
static camera_spi_slave_status_t s_status[CAMERA_SPI_SLAVE_COUNT];
static camera_spi_downlink_payload_t s_downlink[CAMERA_SPI_SLAVE_COUNT];

static uint8 camera_spi_slave_mask(camera_spi_slave_id_t id)
{
    return (uint8)(1U << (uint8)id);
}

static void camera_spi_write_u16_be(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value >> 8);
    buffer[1] = (uint8)(value & 0xFFU);
}

static uint16 camera_spi_read_u16_be(const uint8 *buffer)
{
    return (uint16)(((uint16)buffer[0] << 8) | buffer[1]);
}

static void camera_spi_write_u16_le(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
}

static uint16 camera_spi_read_u16_le(const uint8 *buffer)
{
    return (uint16)(((uint16)buffer[1] << 8) | buffer[0]);
}

static void camera_spi_write_u32_le(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

static uint32 camera_spi_read_u32_le(const uint8 *buffer)
{
    return ((uint32)buffer[0]) |
           ((uint32)buffer[1] << 8) |
           ((uint32)buffer[2] << 16) |
           ((uint32)buffer[3] << 24);
}

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

    memset(buffer, 0xFF, capacity);
    buffer[0] = CAMERA_SPI_FRAME_HEAD_1;
    buffer[1] = CAMERA_SPI_FRAME_HEAD_2;
    buffer[2] = cmd;
    camera_spi_write_u16_be(&buffer[3], payload_len);

    if((payload != NULL) && (payload_len > 0U))
    {
        memcpy(&buffer[5], payload, payload_len);
    }

    crc = camera_spi_crc16(&buffer[2], (uint16)(3U + payload_len));
    buffer[5U + payload_len] = (uint8)(crc & 0xFFU);
    buffer[6U + payload_len] = (uint8)(crc >> 8);
    buffer[7U + payload_len] = CAMERA_SPI_FRAME_TAIL;

    return frame_len;
}

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

static uint16 camera_spi_build_sync_request(camera_spi_slave_id_t id, uint8 *buffer)
{
    uint8 payload_buf[CAMERA_SPI_REQ_PAYLOAD_SIZE];

    camera_spi_serialize_downlink(&s_downlink[id], payload_buf);
    return camera_spi_build_frame(CAMERA_SPI_CMD_SYNC_DATA,
                                  payload_buf,
                                  CAMERA_SPI_REQ_PAYLOAD_SIZE,
                                  buffer,
                                  CAMERA_SPI_TRANSFER_LEN);
}

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

static void camera_spi_refresh_ready_levels(void)
{
    uint8 id;
    uint8 ready_mask = 0U;

    for(id = 0U; id < CAMERA_SPI_SLAVE_COUNT; id++)
    {
        s_status[id].ready_level =
            camera_spi_hw_get_ready_level((camera_spi_hw_slave_id_t)id);
        if(s_status[id].ready_level != 0U)
        {
            ready_mask |= camera_spi_slave_mask((camera_spi_slave_id_t)id);
        }
    }

    s_master.ready_mask = ready_mask;
}

static uint8 camera_spi_pick_next_slave(camera_spi_slave_id_t *id)
{
    uint8 index;
    uint8 candidate;
    uint8 schedule_mask = (uint8)(s_master.ready_mask | s_master.downlink_mask);

    if((id == NULL) || (schedule_mask == 0U))
    {
        return 0U;
    }

    for(index = 0U; index < CAMERA_SPI_SLAVE_COUNT; index++)
    {
        candidate = (uint8)((s_master.round_robin_next + index) % CAMERA_SPI_SLAVE_COUNT);
        if((schedule_mask & camera_spi_slave_mask((camera_spi_slave_id_t)candidate)) != 0U)
        {
            *id = (camera_spi_slave_id_t)candidate;
            s_master.round_robin_next = (uint8)((candidate + 1U) % CAMERA_SPI_SLAVE_COUNT);
            return 1U;
        }
    }

    return 0U;
}

static void camera_spi_apply_result(camera_spi_slave_id_t id,
                                    uint8 ret,
                                    const camera_spi_uplink_payload_t *uplink)
{
    uint8 mask = camera_spi_slave_mask(id);

    s_status[id].ready_level =
        camera_spi_hw_get_ready_level((camera_spi_hw_slave_id_t)id);

    if((ret == CAMERA_SPI_ERR_OK) && (uplink != NULL))
    {
        s_status[id].online = 1U;
        s_status[id].last_error = CAMERA_SPI_ERR_OK;
        s_status[id].ok_count++;
        s_status[id].uplink = *uplink;

        if(uplink->ack_sequence >= s_downlink[id].sequence)
        {
            s_master.downlink_mask &= (uint8)(~mask);
        }
    }
    else
    {
        s_status[id].online = 0U;
        s_status[id].last_error = ret;
        s_status[id].err_count++;
    }
}

static uint8 camera_spi_start_next_transfer(void)
{
    camera_spi_slave_id_t id;
    uint8 ret;
    uint8 mask;

    if(s_master.transfer_busy != 0U)
    {
        return 0U;
    }

    if(camera_spi_pick_next_slave(&id) == 0U)
    {
        return 0U;
    }

    mask = camera_spi_slave_mask(id);
    s_master.ready_mask &= (uint8)(~mask);

    if(camera_spi_build_sync_request(id, s_master.tx_buf) != CAMERA_SPI_REQ_FRAME_LEN)
    {
        camera_spi_apply_result(id, CAMERA_SPI_ERR_DATA_SIZE, NULL);
        return 0U;
    }

    memset(s_master.rx_buf, 0xFF, sizeof(s_master.rx_buf));
    s_master.active_slave = (uint8)id;
    s_master.transfer_poll_guard = 0U;

    ret = camera_spi_hw_start_transfer((camera_spi_hw_slave_id_t)id,
                                       s_master.tx_buf,
                                       s_master.rx_buf,
                                       CAMERA_SPI_TRANSFER_LEN);
    if(ret == CAMERA_SPI_HW_TRANSFER_OK)
    {
        s_master.transfer_busy = 1U;
        return 1U;
    }

    camera_spi_apply_result(id, CAMERA_SPI_ERR_HW, NULL);
    return 0U;
}

static void camera_spi_finish_active_transfer(uint8 forced_timeout)
{
    camera_spi_slave_id_t id = (camera_spi_slave_id_t)s_master.active_slave;
    camera_spi_uplink_payload_t uplink;
    uint8 ret;

    if(id >= CAMERA_SPI_SLAVE_COUNT)
    {
        s_master.transfer_busy = 0U;
        return;
    }

    s_status[id].last_rx_head0 = s_master.rx_buf[0];
    s_status[id].last_rx_head1 = s_master.rx_buf[1];

    if(forced_timeout != 0U)
    {
        camera_spi_hw_abort_transfer();
        ret = CAMERA_SPI_ERR_TIMEOUT;
    }
    else
    {
        ret = camera_spi_hw_finish_transfer();
        if(ret == CAMERA_SPI_HW_TRANSFER_BUSY)
        {
            return;
        }
        if(ret != CAMERA_SPI_HW_TRANSFER_OK)
        {
            ret = CAMERA_SPI_ERR_HW;
        }
        else
        {
            ret = camera_spi_parse_sync_response(s_master.rx_buf,
                                                 CAMERA_SPI_TRANSFER_LEN,
                                                 &uplink);
        }
    }

    s_master.transfer_busy = 0U;

    if(ret == CAMERA_SPI_ERR_OK)
    {
        camera_spi_apply_result(id, ret, &uplink);
    }
    else
    {
        camera_spi_apply_result(id, ret, NULL);
    }
}

void CameraSpi_Init(void)
{
    memset(&s_master, 0, sizeof(s_master));
    memset(s_status, 0, sizeof(s_status));
    memset(s_downlink, 0, sizeof(s_downlink));
    s_master.active_slave = CAMERA_SPI_SLAVE_COUNT;
    camera_spi_hw_init();
}

void CameraSpi_Poll(void)
{
    if(s_master.transfer_busy != 0U)
    {
        s_master.transfer_poll_guard++;
        if(camera_spi_hw_transfer_finished() != 0U)
        {
            camera_spi_finish_active_transfer(0U);
        }
        else if(s_master.transfer_poll_guard > CAMERA_SPI_POLL_TIMEOUT_COUNT)
        {
            camera_spi_finish_active_transfer(1U);
        }
        return;
    }

    camera_spi_refresh_ready_levels();
    (void)camera_spi_start_next_transfer();
}

void CameraSpi_SendRaw(camera_spi_slave_id_t id, const uint8 *data, uint16 len)
{
    uint16 copy_len = len;

    if((id >= CAMERA_SPI_SLAVE_COUNT) || ((data == NULL) && (len > 0U)))
    {
        return;
    }

    if(copy_len > CAMERA_SPI_APP_DATA_CAPACITY)
    {
        copy_len = CAMERA_SPI_APP_DATA_CAPACITY;
    }

    s_downlink[id].sequence++;
    s_downlink[id].length = copy_len;
    memset(s_downlink[id].data, 0, sizeof(s_downlink[id].data));
    if(copy_len > 0U)
    {
        memcpy(s_downlink[id].data, data, copy_len);
    }

    s_master.downlink_mask |= camera_spi_slave_mask(id);
}

uint8 CameraSpi_ReceiveRaw(camera_spi_slave_id_t id, uint8 *data, uint16 *len)
{
    uint16 capacity;
    uint16 copy_len;

    if((id >= CAMERA_SPI_SLAVE_COUNT) || (data == NULL) || (len == NULL))
    {
        return 0U;
    }

    capacity = *len;
    if(capacity > CAMERA_SPI_APP_DATA_CAPACITY)
    {
        capacity = CAMERA_SPI_APP_DATA_CAPACITY;
    }

    copy_len = s_status[id].uplink.length;
    if(copy_len > capacity)
    {
        copy_len = capacity;
    }

    if(copy_len > 0U)
    {
        memcpy(data, s_status[id].uplink.data, copy_len);
    }
    *len = copy_len;

    return (s_status[id].online != 0U) ? 1U : 0U;
}
