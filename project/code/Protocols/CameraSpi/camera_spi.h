/*
 * Camera SPI master public interface.
 *
 * The master polls three 2BL3 image boards. Application code only uses raw
 * payloads; protocol framing and hardware details stay inside the module.
 */

#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include "zf_common_headfile.h"
#include "camera_spi_types.h"

#define CAMERA_SPI_SLAVE_COUNT       (3U)

typedef enum
{
    CAMERA_SPI_SLAVE_1 = 0,
    CAMERA_SPI_SLAVE_2 = 1,
    CAMERA_SPI_SLAVE_3 = 2
} camera_spi_slave_id_t;

void CameraSpi_Init(void);
void CameraSpi_Poll(void);
void CameraSpi_SendRaw(camera_spi_slave_id_t id, const uint8 *data, uint16 len);
uint8 CameraSpi_ReceiveRaw(camera_spi_slave_id_t id, uint8 *data, uint16 *len);

#endif
