#include "camera_spi.h"

#define CAMERA_SPI_FRAME_HEAD_1             (0xAAU)
#define CAMERA_SPI_FRAME_HEAD_2             (0x55U)
#define CAMERA_SPI_FRAME_TAIL               (0xEDU)
#define CAMERA_SPI_CMD_SYNC_DATA            (0x20U)
#define CAMERA_SPI_FRAME_OVERHEAD           (8U)
#define CAMERA_SPI_MAX_DATA_SIZE            (100U)
#define CAMERA_SPI_REQ_META_SIZE            (6U)
#define CAMERA_SPI_RESP_META_SIZE           (12U)
#define CAMERA_SPI_REQ_PAYLOAD_SIZE         (CAMERA_SPI_REQ_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)
#define CAMERA_SPI_RESP_PAYLOAD_SIZE        (CAMERA_SPI_RESP_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)
#define CAMERA_SPI_REQ_FRAME_LEN            (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_REQ_PAYLOAD_SIZE)
#define CAMERA_SPI_RESP_FRAME_LEN           (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_RESP_PAYLOAD_SIZE)
#define CAMERA_SPI_TRANSFER_LEN             CAMERA_SPI_RESP_FRAME_LEN
#define CAMERA_SPI_ALL_SLAVE_MASK           ((uint8)((1U << CAMERA_SPI_SLAVE_COUNT) - 1U))
#define CAMERA_SPI_POLL_TIMEOUT_COUNT       (50000U)

#define CAMERA_SPI_ERR_OK                   (0U)
#define CAMERA_SPI_ERR_FRAME_SHORT          (1U)
#define CAMERA_SPI_ERR_INVALID_HEAD         (2U)
#define CAMERA_SPI_ERR_PAYLOAD_LONG         (3U)
#define CAMERA_SPI_ERR_INVALID_TAIL         (4U)
#define CAMERA_SPI_ERR_CRC                  (5U)
#define CAMERA_SPI_ERR_INCOMPLETE           (6U)
#define CAMERA_SPI_ERR_NULL_PTR             (7U)
#define CAMERA_SPI_ERR_TIMEOUT              (8U)
#define CAMERA_SPI_ERR_DATA_SIZE            (9U)
#define CAMERA_SPI_ERR_INVALID_SLAVE        (10U)
#define CAMERA_SPI_ERR_INVALID_CMD          (11U)
#define CAMERA_SPI_ERR_HW                   (12U)

typedef struct
{
    uint8 transfer_busy;
    uint8 active_slave;
    uint8 round_robin_next;
    volatile uint8 ready_mask;
    uint8 downlink_mask;
    uint32 active_downlink_sequence;
    uint32 transfer_poll_guard;
    uint8 tx_buf[CAMERA_SPI_TRANSFER_LEN];
    uint8 rx_buf[CAMERA_SPI_TRANSFER_LEN];
} camera_spi_master_state_t;

static camera_spi_master_state_t s_camera_spi_master;
static camera_spi_slave_status_t s_camera_spi_status[CAMERA_SPI_SLAVE_COUNT];
static camera_spi_downlink_payload_t s_camera_spi_downlink;
static uint32 s_camera_spi_downlink_synced[CAMERA_SPI_SLAVE_COUNT];
static camera_spi_diag_t s_camera_spi_diag;
static camera_spi_target_t s_camera_spi_target;
static uint32 s_camera_spi_time_ms;

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

    mask = camera_spi_slave_mask(id);
    lock = interrupt_global_disable();
    s_camera_spi_master.ready_mask &= (uint8)(~mask);
    interrupt_global_enable(lock);

    if(camera_spi_build_sync_request(&s_camera_spi_downlink,
                                     s_camera_spi_master.tx_buf) != CAMERA_SPI_REQ_FRAME_LEN)
    {
        camera_spi_apply_result(id, CAMERA_SPI_ERR_DATA_SIZE, NULL);
        return 0U;
    }

    memset(s_camera_spi_master.rx_buf, 0xFF, sizeof(s_camera_spi_master.rx_buf));
    s_camera_spi_master.active_slave = (uint8)id;
    s_camera_spi_master.active_downlink_sequence = s_camera_spi_downlink.sequence;
    s_camera_spi_master.transfer_poll_guard = 0U;

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

    camera_spi_apply_result(id, CAMERA_SPI_ERR_HW, NULL);
    s_camera_spi_diag.transfer_error_count++;
    return 0U;
}

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
            return;
        }
        if(ret != CAMERA_SPI_HW_TRANSFER_OK)
        {
            ret = CAMERA_SPI_ERR_HW;
        }
        else
        {
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
    s_camera_spi_master.downlink_mask = CAMERA_SPI_ALL_SLAVE_MASK;
}
