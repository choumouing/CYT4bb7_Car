#ifndef CAMERA_SPI_HW_H
#define CAMERA_SPI_HW_H

#include "zf_common_headfile.h"

#define CAMERA_SPI_HW_SLAVE_COUNT           (3U)
#define CAMERA_SPI_HW_TRANSFER_OK           (0U)
#define CAMERA_SPI_HW_TRANSFER_BUSY         (1U)
#define CAMERA_SPI_HW_TRANSFER_ERROR        (2U)
#define CAMERA_SPI_HW_TRANSFER_TIMEOUT      (3U)

typedef enum
{
    CAMERA_SPI_HW_SLAVE_1 = 0,
    CAMERA_SPI_HW_SLAVE_2 = 1,
    CAMERA_SPI_HW_SLAVE_3 = 2
} camera_spi_hw_slave_id_t;

void camera_spi_hw_init(void);
uint8 camera_spi_hw_start_transfer(camera_spi_hw_slave_id_t id,
                                   uint8 *tx_buffer,
                                   uint8 *rx_buffer,
                                   uint16 length);
uint8 camera_spi_hw_transfer_finished(void);
uint8 camera_spi_hw_finish_transfer(void);
void camera_spi_hw_abort_transfer(void);
uint8 camera_spi_hw_get_ready_level(camera_spi_hw_slave_id_t id);
void camera_spi_hw_irq_handler(void);

#endif
