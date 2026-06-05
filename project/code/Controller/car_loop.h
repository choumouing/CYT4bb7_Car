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
extern volatile uint8_t timer_25HZ_flag;
extern volatile uint16 g_tick_1000HZ;
extern volatile uint32 tick_1000us_cnt; /* 1000us tick counter */

extern float car_forward_target;
/* 底层麦轮横移命令，闭环速度模式会从右正 m/s 目标转换到该符号。 */
extern float car_strafe_target;
extern uint8 car_control_enabled;
extern uint8 car_emergency_stop_active;

extern volatile float g_air_crsf_std_ch0;
extern volatile float g_air_crsf_std_ch1;
extern volatile float g_air_crsf_std_ch2;
extern volatile float g_air_crsf_std_ch3;
extern volatile float g_air_crsf_std_ch4;
extern volatile float g_air_crsf_std_ch5;
extern volatile float g_air_crsf_std_ch6;
extern volatile float g_air_crsf_std_ch7;

typedef struct
{
    volatile uint8 valid;
    volatile float x;      /* Image x, left origin. */
    volatile float y;      /* Image y, top origin. */
    volatile float area;   /* Connected-component area in pixels. */
} car_image_spi_target_t;

typedef struct
{
    volatile uint8 valid;
    volatile float cx;     /* Image center x, left origin. */
    volatile float cy;     /* Image center y, top origin. */
    volatile float width;
    volatile float length;
    volatile float angle;
} car_image_spi_car_lamp_t;

typedef struct
{
    volatile uint8 online;
    volatile uint8 rx_len;
    volatile uint8 protocol_version;
    volatile uint8 beacon_count;
    volatile uint8 car_lamp_count;
    volatile car_image_spi_target_t target[CAMERA_SPI_IMAGE_TARGET_COUNT];
    volatile car_image_spi_car_lamp_t car_lamp;
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
