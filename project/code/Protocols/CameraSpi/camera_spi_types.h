/*********************************************************************************************************************
 * @file    camera_spi_types.h
 * @brief   Camera SPI shared payload and typed data definitions.
 * @note    Keep the wire-facing constants identical on master and slave.
 ********************************************************************************************************************/

#ifndef CAMERA_SPI_TYPES_H
#define CAMERA_SPI_TYPES_H

#include "zf_common_typedef.h"

#define CAMERA_SPI_IMAGE_TARGET_COUNT           (5U)
#define CAMERA_SPI_IMAGE_TARGET_VALID_OFFSET    (0U)
#define CAMERA_SPI_IMAGE_TARGET_X_OFFSET        (1U)
#define CAMERA_SPI_IMAGE_TARGET_Y_OFFSET        (5U)
#define CAMERA_SPI_IMAGE_TARGET_RADIUS_OFFSET   (9U)
#define CAMERA_SPI_IMAGE_TARGET_RESERVED_OFFSET (13U)
#define CAMERA_SPI_IMAGE_TARGET_RESERVED_SIZE   (4U)
#define CAMERA_SPI_IMAGE_TARGET_SLOT_SIZE       (17U)
#define CAMERA_SPI_IMAGE_TARGET_PACKET_SIZE \
    (CAMERA_SPI_IMAGE_TARGET_COUNT * CAMERA_SPI_IMAGE_TARGET_SLOT_SIZE)

#ifndef CAMERA_SPI_APP_DATA_CAPACITY
#define CAMERA_SPI_APP_DATA_CAPACITY    CAMERA_SPI_IMAGE_TARGET_PACKET_SIZE
#endif

#define CAMERA_SPI_UPLINK_FLAG_PENDING      (0x01U)
#define CAMERA_SPI_UPLINK_FLAG_DOWNLINK_NEW (0x02U)

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

#endif
