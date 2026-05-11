#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"

#define CAMERA_SPI_SLAVE_COUNT              (3U)
#define CAMERA_SPI_APP_DATA_CAPACITY        (12U)

typedef enum
{
    CAMERA_SPI_SLAVE_1 = 0,
    CAMERA_SPI_SLAVE_2 = 1,
    CAMERA_SPI_SLAVE_3 = 2
} camera_spi_slave_id_t;

typedef struct
{
    uint16 length;
    uint8 data[CAMERA_SPI_APP_DATA_CAPACITY];
} camera_spi_payload_buffer_t;

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
    uint8 int_level;
    uint8 last_error;
    uint32 ok_count;
    uint32 err_count;
    uint32 last_update_ms;
    camera_spi_uplink_payload_t uplink;
} camera_spi_slave_status_t;

typedef struct
{
    uint8 valid;
    uint8 camera_id;
    uint16 frame_id;
    uint16 spot_count;
    uint16 spot_index;
    uint16 x;
    uint16 y;
    uint16 area;
    uint32 uplink_sequence;
    uint32 last_update_ms;
    uint32 age_ms;
} camera_spi_target_t;

typedef struct
{
    uint8 transfer_busy;
    uint8 active_slave;
    uint8 ready_mask;
    uint8 downlink_mask;
    uint8 last_error;
    uint32 poll_count;
    uint32 transfer_start_count;
    uint32 transfer_done_count;
    uint32 transfer_error_count;
    uint32 timeout_count;
    uint32 ready_irq_count[CAMERA_SPI_SLAVE_COUNT];
} camera_spi_diag_t;

void camera_spi_init(void);
void camera_spi_poll(void);
void camera_spi_update_100HZ(uint32 system_time_ms);
void camera_spi_notify_ready(camera_spi_slave_id_t id);
uint8 camera_spi_get_status(camera_spi_slave_id_t id, camera_spi_slave_status_t *status);
uint8 camera_spi_get_diag(camera_spi_diag_t *diag);
uint8 camera_spi_get_target(camera_spi_target_t *target);
void camera_spi_set_downlink_payload(const camera_spi_payload_buffer_t *payload);

#endif
