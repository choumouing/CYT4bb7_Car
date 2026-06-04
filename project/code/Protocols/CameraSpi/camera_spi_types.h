/*********************************************************************************************************************
 * @file    camera_spi_types.h
 * @brief   Camera SPI shared payload and typed data definitions.
 * @note    Keep the wire-facing constants identical on master and slave.
 ********************************************************************************************************************/

#ifndef CAMERA_SPI_TYPES_H
#define CAMERA_SPI_TYPES_H

#include "zf_common_typedef.h"

#define CAMERA_SPI_IMAGE_PROTOCOL_VERSION          (2U)
#define CAMERA_SPI_IMAGE_BEACON_COUNT              (4U)
#define CAMERA_SPI_IMAGE_CAR_LAMP_COUNT            (1U)

#define CAMERA_SPI_IMAGE_VERSION_OFFSET            (0U)
#define CAMERA_SPI_IMAGE_BEACON_COUNT_OFFSET       (1U)
#define CAMERA_SPI_IMAGE_CAR_LAMP_COUNT_OFFSET     (2U)
#define CAMERA_SPI_IMAGE_HEADER_RESERVED_OFFSET    (3U)
#define CAMERA_SPI_IMAGE_HEADER_SIZE               (4U)

#define CAMERA_SPI_IMAGE_BEACON_VALID_OFFSET       (0U)
#define CAMERA_SPI_IMAGE_BEACON_X_OFFSET           (1U)  /* float LE: E-disk algorithm x, image-center origin */
#define CAMERA_SPI_IMAGE_BEACON_Y_OFFSET           (5U)  /* float LE: E-disk algorithm y, image-center origin */
#define CAMERA_SPI_IMAGE_BEACON_RADIUS_OFFSET      (9U)  /* float LE: equivalent radius in pixels */
#define CAMERA_SPI_IMAGE_BEACON_SLOT_SIZE          (13U)

#define CAMERA_SPI_IMAGE_CAR_LAMP_VALID_OFFSET     (0U)
#define CAMERA_SPI_IMAGE_CAR_LAMP_CX_OFFSET        (1U)  /* float LE: E-disk algorithm cx, image-center origin */
#define CAMERA_SPI_IMAGE_CAR_LAMP_CY_OFFSET        (5U)  /* float LE: E-disk algorithm cy, image-center origin */
#define CAMERA_SPI_IMAGE_CAR_LAMP_WIDTH_OFFSET     (9U)
#define CAMERA_SPI_IMAGE_CAR_LAMP_LENGTH_OFFSET    (13U)
#define CAMERA_SPI_IMAGE_CAR_LAMP_ANGLE_OFFSET     (17U)
#define CAMERA_SPI_IMAGE_CAR_LAMP_SLOT_SIZE        (21U)

#define CAMERA_SPI_IMAGE_BEACON_PACKET_OFFSET      CAMERA_SPI_IMAGE_HEADER_SIZE
#define CAMERA_SPI_IMAGE_CAR_LAMP_PACKET_OFFSET \
    (CAMERA_SPI_IMAGE_BEACON_PACKET_OFFSET + \
     (CAMERA_SPI_IMAGE_BEACON_COUNT * CAMERA_SPI_IMAGE_BEACON_SLOT_SIZE))
#define CAMERA_SPI_IMAGE_PACKET_SIZE \
    (CAMERA_SPI_IMAGE_CAR_LAMP_PACKET_OFFSET + \
     (CAMERA_SPI_IMAGE_CAR_LAMP_COUNT * CAMERA_SPI_IMAGE_CAR_LAMP_SLOT_SIZE))

#define CAMERA_SPI_IMAGE_TARGET_COUNT              CAMERA_SPI_IMAGE_BEACON_COUNT
#define CAMERA_SPI_IMAGE_TARGET_PACKET_SIZE        CAMERA_SPI_IMAGE_PACKET_SIZE

#define CAMERA_SPI_DOWNLINK_MAGIC                  (0x5AU)
#define CAMERA_SPI_DOWNLINK_MAGIC_OFFSET           (0U)
#define CAMERA_SPI_DOWNLINK_BOARD_ID_OFFSET        (1U)
#define CAMERA_SPI_DOWNLINK_COUNTER_OFFSET         (2U)
#define CAMERA_SPI_BOARD_COUNT                     (3U)

#ifndef CAMERA_SPI_APP_DATA_CAPACITY
#define CAMERA_SPI_APP_DATA_CAPACITY    CAMERA_SPI_IMAGE_PACKET_SIZE
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
