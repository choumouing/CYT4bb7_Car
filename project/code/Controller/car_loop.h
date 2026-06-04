/*
 * 模块职责：主循环调度器。
 * PIT 中断只置标志位，car_loop_poll() 在主循环中清标志并执行周期任务。
 * 同时汇聚三路 CYT2BL3 摄像头 SPI 的信标检测结果。
 *
 * 调用节奏：
 *   car_loop_init() 在系统启动时调用一次。
 *   car_loop_poll() 在 while(1) 主循环中轮询，内部按 1000/100/25 Hz 分频执行。
 *
 * 数据来源：
 *   三块 CYT2BL3 从机板通过 SPI 上报信标坐标，board[0..2] 对应摄像头 1~3。
 *   遥控通道来自无人机 CRSF 转发的 ch0~ch7。
 */

#ifndef CAR_LOOP_H
#define CAR_LOOP_H

#include "zf_common_headfile.h"
#include "Protocols/CameraSpi/camera_spi_types.h"

/* SPI 从机板数量（3 路摄像头各一块 CYT2BL3） */
#define CAR_IMAGE_SPI_BOARD_COUNT CAMERA_SPI_BOARD_COUNT
/* 单次 SPI 下行原始数据长度（字节） */
#define CAR_IMAGE_SPI_TX_RAW_SIZE (12U)
/* 单次 SPI 上行原始数据长度（字节），跟随协议定义 */
#define CAR_IMAGE_SPI_RAW_SIZE    CAMERA_SPI_IMAGE_TARGET_PACKET_SIZE

/* 100Hz 周期任务标志，PIT 中断置 1，poll 中清 0 */
extern volatile uint8_t timer_100HZ_flag;
/* 25Hz 周期任务标志 */
extern volatile uint8_t timer_25HZ_flag;
/* 1000Hz 节拍计数，用于超时判断和在线状态老化 */
extern volatile uint16 g_tick_1000HZ;
/* 1ms tick 累计计数 */
extern volatile uint32 tick_1000us_cnt;

/* 前进速度目标，单位 m/s，正方向为车头朝向 */
extern float car_forward_target;
/* 横移速度目标，单位 m/s，正方向为车身右侧 */
extern float car_strafe_target;
/* 控制使能标志：1=允许电机输出，0=停止输出 */
extern uint8 car_control_enabled;
/* 急停激活标志：1=急停生效，电机强制停止 */
extern uint8 car_emergency_stop_active;

/* 遥控通道标准化值（来自无人机 CRSF 转发） */
/* 范围 -1.0 ~ +1.0，ch0=油门/前后，ch1=横移，ch2=航向，ch3=辅助等 */
extern volatile float g_air_crsf_std_ch0;
extern volatile float g_air_crsf_std_ch1;
extern volatile float g_air_crsf_std_ch2;
extern volatile float g_air_crsf_std_ch3;
extern volatile float g_air_crsf_std_ch4;
extern volatile float g_air_crsf_std_ch5;
extern volatile float g_air_crsf_std_ch6;
extern volatile float g_air_crsf_std_ch7;

/* 信标目标检测结果（单个目标） */
/* 坐标系：以图像中心为原点，单位像素 */
typedef struct
{
    volatile uint8 valid;  /* 1=有效目标，0=无效/未检测到 */
    volatile float x;      /* 目标中心 x 坐标（图像中心原点） */
    volatile float y;      /* 目标中心 y 坐标（图像中心原点） */
    volatile float radius; /* 等效连通域半径，单位像素，反映目标大小 */
} car_image_spi_target_t;

/* 车灯检测结果 */
/* 坐标系：以图像中心为原点，单位像素 */
typedef struct
{
    volatile uint8 valid;  /* 1=有效车灯，0=无效 */
    volatile float cx;     /* 车灯中心 x 坐标 */
    volatile float cy;     /* 车灯中心 y 坐标 */
    volatile float width;  /* 车灯宽度，单位像素 */
    volatile float length; /* 车灯长度，单位像素 */
    volatile float angle;  /* 车灯朝向角度，单位度 */
} car_image_spi_car_lamp_t;

/* 单块 CYT2BL3 从机板的通信状态 */
typedef struct
{
    volatile uint8 online;            /* 1=在线（最近一次 SPI 收到有效数据） */
    volatile uint8 rx_len;            /* 最近一次上行 payload 长度 */
    volatile uint8 protocol_version;  /* 协议版本号 */
    volatile uint8 beacon_count;      /* 本次上报的信标目标数量 */
    volatile uint8 car_lamp_count;    /* 本次上报的车灯数量 */
    volatile car_image_spi_target_t target[CAMERA_SPI_IMAGE_TARGET_COUNT]; /* 信标目标数组 */
    volatile car_image_spi_car_lamp_t car_lamp; /* 车灯检测结果（每板最多 1 个） */
    volatile uint32 rx_count;         /* 累计成功接收次数 */
    volatile uint32 miss_count;       /* 累计超时/丢失次数 */
} car_image_spi_board_t;

/* 三路摄像头 SPI 汇聚状态 */
typedef struct
{
    volatile uint32 tx_counter;                        /* 下行帧序号 */
    volatile car_image_spi_board_t board[CAR_IMAGE_SPI_BOARD_COUNT]; /* 三块从机板状态 */
} car_image_spi_state_t;

/* 全局三路摄像头 SPI 状态 */
extern volatile car_image_spi_state_t g_image_spi;

/* 初始化：清零所有状态和标志位 */
void car_loop_init(void);
/* 主循环轮询：内部按 1000/100/25Hz 分频执行各周期任务 */
void car_loop_poll(void);

#endif
