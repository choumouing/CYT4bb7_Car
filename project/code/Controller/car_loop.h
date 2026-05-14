/*
 * Main loop scheduler.
 *
 * PIT interrupt only sets flags. car_loop_poll() clears flags and runs
 * periodic tasks in the main loop context.
 */

#ifndef CAR_LOOP_H
#define CAR_LOOP_H

#include "zf_common_headfile.h"

#define CAR_IMAGE_SPI_BOARD_COUNT (3U)
#define CAR_IMAGE_SPI_RAW_SIZE    (12U)

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
    volatile uint8 online;
    volatile uint8 rx_len;
    volatile uint16 frame_id;
    volatile uint16 spot_count;
    volatile uint16 x;
    volatile uint16 y;
    volatile uint16 area;
    volatile uint32 rx_count;
    volatile uint32 miss_count;
    volatile uint8 raw[CAR_IMAGE_SPI_RAW_SIZE];
} car_image_spi_board_t;

typedef struct
{
    volatile uint32 tx_counter;
    volatile uint8 next_send_id;
    volatile car_image_spi_board_t board[CAR_IMAGE_SPI_BOARD_COUNT];
} car_image_spi_state_t;

extern volatile car_image_spi_state_t g_image_spi;

void car_loop_init(void);
void car_loop_poll(void);

#endif
