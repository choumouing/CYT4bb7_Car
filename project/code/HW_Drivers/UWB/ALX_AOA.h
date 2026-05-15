#ifndef _ALX_AOA_H_
#define _ALX_AOA_H_

#include "zf_common_typedef.h"
#include "zf_driver_uart.h"

#define ALX_AOA_UART_INDEX                 (UART_1)
#define ALX_AOA_UART_TX_PIN                (UART1_TX_P04_1)
#define ALX_AOA_UART_RX_PIN                (UART1_RX_P04_0)
#define ALX_AOA_UART_BAUDRATE              (115200)

#define ALX_AOA_RX_BUFFER_SIZE             (256)
#define ALX_AOA_MAX_FRAME_SIZE             (64)

#define ALX_AOA_FRAME_HEADER_BYTE          (0xFF)
#define ALX_AOA_CMD_POSITION               (0x2001)
#define ALX_AOA_POSITION_FRAME_SIZE        (37)

#define ALX_AOA_DIST_MIN_CM                (20)
#define ALX_AOA_DIST_MAX_CM                (500)
#define ALX_AOA_EL_MAX_DEG                 (90)
#define ALX_AOA_AB_ALPHA                   (0.34f)
#define ALX_AOA_AB_BETA                    (0.016f)
#define ALX_AOA_VEL_DECAY                  (0.88f)
#define ALX_AOA_EL_GATE_DEG                (60)
#define ALX_AOA_GATE_BASE_CM               (35.0f)
#define ALX_AOA_GATE_SPEED_CM_S            (250.0f)
#define ALX_AOA_GATE_REACQUIRE_COUNT       (12U)

typedef struct
{
    uint32 tag_id;
    uint32 base_id;
    uint32 distance_cm;
    int32  x_cm;
    int32  y_cm;
    int32  z_cm;
    int16  azimuth_deg;
    int16  elevation_deg;
    uint32 last_position_ms;
    uint8  valid;
}ALX_AOA_Position_t;

extern float alx_aoa_x_cm;
extern float alx_aoa_y_cm;
extern float alx_aoa_debug_accepted;
extern float alx_aoa_debug_jump_cm;
extern float alx_aoa_debug_gate_cm;
extern uint32 alx_aoa_debug_rx_bytes;
extern uint32 alx_aoa_debug_rx_overflow;
extern uint32 alx_aoa_debug_frame_headers;
extern uint32 alx_aoa_debug_frame_ok;
extern uint32 alx_aoa_debug_frame_error;

void  ALX_AOA_Init              (void);
void  ALX_AOA_Reset             (void);
void  ALX_AOA_InputByte         (uint8 dat);
uint8 ALX_AOA_Update            (uint32 now_ms);
uint8 ALX_AOA_Update_25HZ       (uint32 now_ms);
uint8 ALX_AOA_GetLatest         (ALX_AOA_Position_t *data);
uint8 ALX_AOA_GetFilteredXY     (float *x_cm, float *y_cm);
uint8 ALX_AOA_IsTagOnline       (uint32 now_ms, uint32 timeout_ms);

#endif
