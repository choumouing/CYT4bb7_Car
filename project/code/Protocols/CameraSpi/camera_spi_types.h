/*********************************************************************************************************************
 * @file    camera_spi_types.h
 * @brief   Camera SPI shared payload and typed data definitions.
 * @note    Keep the wire-facing constants identical on master and slave.
 ********************************************************************************************************************/

#ifndef CAMERA_SPI_TYPES_H
#define CAMERA_SPI_TYPES_H

#include "zf_common_typedef.h"

#ifndef CAMERA_SPI_APP_FIELD_SIZE
#define CAMERA_SPI_APP_FIELD_SIZE       (4U)
#endif

#ifndef CAMERA_SPI_APP_FIELD_COUNT
#define CAMERA_SPI_APP_FIELD_COUNT      (3U)
#endif

#ifndef CAMERA_SPI_APP_DATA_CAPACITY
#define CAMERA_SPI_APP_DATA_CAPACITY    (CAMERA_SPI_APP_FIELD_SIZE * CAMERA_SPI_APP_FIELD_COUNT)
#endif

#define CAMERA_SPI_UPLINK_FLAG_PENDING      (0x01U)
#define CAMERA_SPI_UPLINK_FLAG_DOWNLINK_NEW (0x02U)

#define CAMERA_SPI_TYPED_KIND_RAW       (0x00U)
#define CAMERA_SPI_TYPED_KIND_HEARTBEAT (0xA5U)
#define CAMERA_SPI_TYPED_KIND_TARGET    (0x31U)
#define CAMERA_SPI_TYPED_KIND_PARAM     (0x32U)

#define CAMERA_SPI_HEARTBEAT_MAGIC      (0x4353U)
#define CAMERA_SPI_HEARTBEAT_VERSION    (1U)

#ifndef CAMERA_SPI_H
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
#endif

typedef struct
{
    uint16 magic;
    uint8 version;
    uint8 flags;
    uint32 counter;
    uint16 timestamp_ms;
    uint8 status;
    uint8 reserved;
} camera_spi_heartbeat_t;

typedef struct
{
    uint16 frame_id;
    uint16 spot_count;
    uint16 spot_index;
    uint16 x;
    uint16 y;
    uint16 area;
} camera_spi_target_payload_t;

typedef struct
{
    uint8 kind;
    union
    {
        camera_spi_heartbeat_t heartbeat;
        camera_spi_target_payload_t target;
        camera_spi_payload_buffer_t raw;
    } value;
} camera_spi_typed_payload_t;

#endif
