/*********************************************************************************************************************
 * @file    camera_spi_protocol.h
 * @brief   Camera SPI frame constants and small endian helpers.
 ********************************************************************************************************************/

#ifndef CAMERA_SPI_PROTOCOL_H
#define CAMERA_SPI_PROTOCOL_H

#include "camera_spi_types.h"

#define CAMERA_SPI_FRAME_HEAD_1             (0xAAU)
#define CAMERA_SPI_FRAME_HEAD_2             (0x55U)
#define CAMERA_SPI_FRAME_TAIL               (0xEDU)
#define CAMERA_SPI_CMD_SYNC_DATA            (0x20U)

#define CAMERA_SPI_MAX_DATA_SIZE            (100U)
#define CAMERA_SPI_FRAME_OVERHEAD           (8U)
#define CAMERA_SPI_REQ_META_SIZE            (6U)
#define CAMERA_SPI_RESP_META_SIZE           (12U)
#define CAMERA_SPI_REQ_PAYLOAD_SIZE         (CAMERA_SPI_REQ_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)
#define CAMERA_SPI_RESP_PAYLOAD_SIZE        (CAMERA_SPI_RESP_META_SIZE + CAMERA_SPI_APP_DATA_CAPACITY)
#define CAMERA_SPI_REQ_FRAME_LEN            (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_REQ_PAYLOAD_SIZE)
#define CAMERA_SPI_RESP_FRAME_LEN           (CAMERA_SPI_FRAME_OVERHEAD + CAMERA_SPI_RESP_PAYLOAD_SIZE)
#define CAMERA_SPI_TRANSFER_LEN             CAMERA_SPI_RESP_FRAME_LEN

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
#define CAMERA_SPI_ERR_OVERFLOW             (13U)
#define CAMERA_SPI_ERR_UNDERFLOW            (14U)
#define CAMERA_SPI_ERR_NOT_READY            (15U)

static inline void camera_spi_write_u16_be(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value >> 8);
    buffer[1] = (uint8)(value & 0xFFU);
}

static inline uint16 camera_spi_read_u16_be(const uint8 *buffer)
{
    return (uint16)(((uint16)buffer[0] << 8) | buffer[1]);
}

static inline void camera_spi_write_u16_le(uint8 *buffer, uint16 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
}

static inline uint16 camera_spi_read_u16_le(const uint8 *buffer)
{
    return (uint16)(((uint16)buffer[1] << 8) | buffer[0]);
}

static inline void camera_spi_write_u32_le(uint8 *buffer, uint32 value)
{
    buffer[0] = (uint8)(value & 0xFFU);
    buffer[1] = (uint8)((value >> 8) & 0xFFU);
    buffer[2] = (uint8)((value >> 16) & 0xFFU);
    buffer[3] = (uint8)((value >> 24) & 0xFFU);
}

static inline uint32 camera_spi_read_u32_le(const uint8 *buffer)
{
    return ((uint32)buffer[0]) |
           ((uint32)buffer[1] << 8) |
           ((uint32)buffer[2] << 16) |
           ((uint32)buffer[3] << 24);
}

static inline uint16 camera_spi_crc16(const uint8 *data, uint16 len)
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

#endif
