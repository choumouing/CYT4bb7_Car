#ifndef AIR_COMM_CAR_H
#define AIR_COMM_CAR_H

#include "zf_common_headfile.h"

#define AIR_COMM_PARAM_NAME_MAX             (16U)
#define AIR_COMM_FUNC_PARAMS_MAX            (8U)
#define AIR_COMM_RUN_DATA_MAX_FLOATS        (32U)
#define AIR_COMM_BAUDRATE                   (1152000U)

#define AIR_COMM_STATUS_OK                  (0U)
#define AIR_COMM_STATUS_NOT_FOUND           (1U)
#define AIR_COMM_STATUS_OUT_OF_RANGE        (2U)
#define AIR_COMM_STATUS_ERROR               (3U)

typedef void (*air_comm_run_data_fn)(const float *data, uint8 count);

typedef struct
{
    uint32 tick_ms;
    uint32 tx_frame_count;
    uint32 tx_byte_count;
    uint32 rx_frame_count;
    uint32 rx_byte_count;
    uint32 rx_raw_byte_count;
    uint32 crc_error_count;
    uint32 rx_oversize_count;
    uint32 rx_queue_overflow_count;
    uint32 ack_ok_count;
    uint32 ack_timeout_count;
    uint32 ack_retry_count;
    uint32 heartbeat_tx_count;
    uint32 heartbeat_rx_count;
    uint8 online_status;
    uint8 pending_ack;
    uint8 last_ack_status;
} air_comm_stats_t;

void air_comm_car_init(void);
void air_comm_car_tick_1MS(void);
void air_comm_car_poll(void);
void air_comm_car_update_100HZ(void);
void air_comm_car_rx_byte(uint8 byte);
uint8 air_comm_car_is_online(void);
uint8 air_comm_car_get_online_status(void);
uint32 air_comm_car_get_tick(void);
uint8 air_comm_car_set_param(const char *name, float value);
uint8 air_comm_car_exec_func(uint8 func_id);
void air_comm_car_set_run_data_callback(air_comm_run_data_fn callback);
void air_comm_car_get_stats(air_comm_stats_t *stats);

#endif
