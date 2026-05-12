#ifndef CAMERA_SPI_HW_H
#define CAMERA_SPI_HW_H

#include "zf_common_headfile.h"

/*
 * Camera SPI 硬件传输驱动
 * - 使用 SPI_0（SCB7）主模式，10MHz，CPOL0_CPHA0
 * - 支持 3 个从设备（3 个摄像头模块），通过独立 CS 引脚选择
 * - 中断驱动的非阻塞传输：start -> 轮询 finished -> finish
 * - 从设备就绪信号通过 INT 引脚（EXTI 上升沿）检测
 */

#define CAMERA_SPI_HW_SLAVE_COUNT           (3U)   /* 从设备数量 */
#define CAMERA_SPI_HW_TRANSFER_OK           (0U)   /* 传输成功 */
#define CAMERA_SPI_HW_TRANSFER_BUSY         (1U)   /* 传输中 / 未完成 */
#define CAMERA_SPI_HW_TRANSFER_ERROR        (2U)   /* 传输错误（参数错或硬件错） */
#define CAMERA_SPI_HW_TRANSFER_TIMEOUT      (3U)   /* 超时（预留） */

/* 从设备 ID，对应 CS 和 INT 引脚数组下标 */
typedef enum
{
    CAMERA_SPI_HW_SLAVE_1 = 0,  /* CS=P02_3, INT=P02_4 */
    CAMERA_SPI_HW_SLAVE_2 = 1,  /* CS=P01_0, INT=P01_1 */
    CAMERA_SPI_HW_SLAVE_3 = 2   /* CS=P19_0, INT=P19_1 */
} camera_spi_hw_slave_id_t;

/* 初始化 SPI_0 + SCB7 + CS/INT GPIO + EXTI，上电调用一次 */
void camera_spi_hw_init(void);

/*
 * 启动一次 SPI 传输（非阻塞）
 * id: 从设备号；tx_buffer/rx_buffer: 发送/接收缓冲区（DMA 方向，必须持久有效）
 * length: 传输字节数
 * 返回 CAMERA_SPI_HW_TRANSFER_OK 表示已启动，BUSY 表示上一次未完成
 */
uint8 camera_spi_hw_start_transfer(camera_spi_hw_slave_id_t id,
                                   uint8 *tx_buffer,
                                   uint8 *rx_buffer,
                                   uint16 length);

/* 查询传输是否完成：1=完成（含错误完成），0=仍在进行 */
uint8 camera_spi_hw_transfer_finished(void);

/*
 * 结束传输：拉高 CS，释放 busy 标志
 * 返回传输结果：OK / ERROR / BUSY（传输未完成则返回 BUSY）
 */
uint8 camera_spi_hw_finish_transfer(void);

/* 强制中止传输（busy 时调用），拉高 CS 并复位状态 */
void camera_spi_hw_abort_transfer(void);

/* 读取从设备就绪引脚电平（INT 引脚），1=就绪可通信 */
uint8 camera_spi_hw_get_ready_level(camera_spi_hw_slave_id_t id);

/* SCB7 SPI 中断处理函数（在中断向量表中注册） */
void camera_spi_hw_irq_handler(void);

#endif
