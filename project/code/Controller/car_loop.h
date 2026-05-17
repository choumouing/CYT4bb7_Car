/*
 * Main loop scheduler.
 *
 * PIT interrupt only sets flags. car_loop_poll() clears flags and runs
 * periodic tasks in the main loop context.
 */

#ifndef CAR_LOOP_H
#define CAR_LOOP_H

#include "zf_common_headfile.h"
#include "Protocols/CameraSpi/camera_spi_types.h"

#define CAR_IMAGE_SPI_BOARD_COUNT (3U)
#define CAR_IMAGE_SPI_TX_RAW_SIZE (12U)
#define CAR_IMAGE_SPI_RAW_SIZE    CAMERA_SPI_IMAGE_TARGET_PACKET_SIZE

extern volatile uint8_t timer_100HZ_flag;
extern volatile uint8_t timer_50HZ_flag;
extern volatile uint8_t timer_25HZ_flag;
extern volatile uint16 g_tick_1000HZ;

extern float car_forward_target;
extern float car_strafe_target;
extern float car_rotate_target;
extern uint8 car_control_enabled;
extern uint8 car_emergency_stop_active;

typedef struct
{
    volatile uint8 valid;
    volatile float x;
    volatile float y;
    volatile float radius;
} car_image_spi_target_t;

typedef struct
{
    volatile uint8 online;
    volatile uint8 rx_len;
    volatile car_image_spi_target_t target[CAMERA_SPI_IMAGE_TARGET_COUNT];
    volatile uint32 rx_count;
    volatile uint32 miss_count;
} car_image_spi_board_t;

typedef struct
{
    volatile uint32 tx_counter;
    volatile car_image_spi_board_t board[CAR_IMAGE_SPI_BOARD_COUNT];
} car_image_spi_state_t;

extern volatile car_image_spi_state_t g_image_spi;

void car_loop_init(void);
void car_loop_poll(void);

#endif
