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
#define ALX_AOA_CMD_HEARTBEAT              (0x2002)
#define ALX_AOA_POSITION_FRAME_SIZE        (37)
#define ALX_AOA_HEARTBEAT_FRAME_SIZE       (16)

#define ALX_AOA_DIST_MIN_CM                (20)
#define ALX_AOA_DIST_MAX_CM                (500)
#define ALX_AOA_EL_MAX_DEG                 (65)
#define ALX_AOA_XY_GATE_X_CM               (30.0f)
#define ALX_AOA_XY_GATE_Y_CM               (30.0f)
#define ALX_AOA_XY_GATE_2D_CM              (50.0f)
#define ALX_AOA_AB_ALPHA                   (0.40f)
#define ALX_AOA_AB_BETA                    (0.020f)
#define ALX_AOA_VEL_DECAY                  (0.90f)
#define ALX_AOA_MEDIAN_SIZE                (3U)
#define ALX_AOA_REACQUIRE_COUNT            (3U)

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

typedef ALX_AOA_Position_t ALX_AOA_Data_t;

typedef struct
{
    uint32 base_id;
    uint32 sequence;
    uint32 last_heartbeat_ms;
    uint8  valid;
}ALX_AOA_Heartbeat_t;

typedef struct
{
    uint32 rx_bytes;
    uint32 rx_overflow;
    uint8  rx_last_byte;
    uint16 parser_frame_length;
    uint16 last_packet_length;
    uint16 last_command;
    uint32 frame_total;
    uint32 position_count;
    uint32 heartbeat_count;
    uint32 frame_bad_header;
    uint32 frame_bad_length;
    uint32 frame_bad_xor;
    uint32 frame_unknown_cmd;
}ALX_AOA_Stats_t;

extern float alx_aoa_x_cm;
extern float alx_aoa_y_cm;

void  ALX_AOA_Init              (void);
void  ALX_AOA_Reset             (void);
void  ALX_AOA_InputByte         (uint8 dat);
void  ALX_AOA_InputBytes        (const uint8 *dat, uint16 length);
uint8 ALX_AOA_Update            (uint32 now_ms);
uint8 ALX_AOA_GetLatest         (ALX_AOA_Position_t *data);
uint8 ALX_AOA_GetFilteredXY     (float *x_cm, float *y_cm);
uint8 ALX_AOA_GetHeartbeat      (ALX_AOA_Heartbeat_t *data);
uint8 ALX_AOA_IsTagOnline       (uint32 now_ms, uint32 timeout_ms);
void  ALX_AOA_GetStats          (ALX_AOA_Stats_t *stats);
void  ALX_AOA_ResetStats        (void);

#endif
