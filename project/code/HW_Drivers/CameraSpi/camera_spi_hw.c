#include "camera_spi_hw.h"

#include "scb/cy_scb_spi.h"

#define CAMERA_SPI_HW_CHANNEL               SPI_0
#define CAMERA_SPI_HW_BAUD                  (10000000U)
#define CAMERA_SPI_HW_CLK                   SPI0_CLK_P02_2
#define CAMERA_SPI_HW_MOSI                  SPI0_MOSI_P02_1
#define CAMERA_SPI_HW_MISO                  SPI0_MISO_P02_0
#define CAMERA_SPI_HW_SCB                   SCB7
#define CAMERA_SPI_HW_SCB_IRQ               scb_7_interrupt_IRQn
#define CAMERA_SPI_HW_CPU_IRQ               CPUIntIdx5_IRQn
#define CAMERA_SPI_HW_IRQ_PRIORITY          (4U)
#define CAMERA_SPI_HW_CS_SETUP_DELAY_US     (10U)

typedef struct
{
    cy_stc_scb_spi_context_t context;
    volatile uint8 busy;
    camera_spi_hw_slave_id_t active_slave;
} camera_spi_hw_state_t;

static const gpio_pin_enum s_camera_spi_cs_pins[CAMERA_SPI_HW_SLAVE_COUNT] =
{
    P02_3,
    P01_0,
    P19_0
};

static const gpio_pin_enum s_camera_spi_int_pins[CAMERA_SPI_HW_SLAVE_COUNT] =
{
    P02_4,
    P01_1,
    P19_1
};

static camera_spi_hw_state_t s_camera_spi_hw;

static void camera_spi_hw_build_config(cy_stc_scb_spi_config_t *config)
{
    memset(config, 0, sizeof(*config));

    config->spiMode = CY_SCB_SPI_MASTER;
    config->subMode = CY_SCB_SPI_MOTOROLA;
    config->oversample = 4U;
    config->rxDataWidth = 8U;
    config->txDataWidth = 8U;
    config->enableMsbFirst = true;
    config->enableMisoLateSample = true;
    config->sclkMode = CY_SCB_SPI_CPHA0_CPOL0;
}

void camera_spi_hw_irq_handler(void)
{
    Cy_SCB_SPI_Interrupt(CAMERA_SPI_HW_SCB, &s_camera_spi_hw.context);
}

void camera_spi_hw_init(void)
{
    cy_stc_scb_spi_config_t spi_config;
    cy_stc_sysint_irq_t spi_irq_config;
    uint8 index;

    spi_init(CAMERA_SPI_HW_CHANNEL,
             SPI_MODE0,
             CAMERA_SPI_HW_BAUD,
             CAMERA_SPI_HW_CLK,
             CAMERA_SPI_HW_MOSI,
             CAMERA_SPI_HW_MISO,
             SPI_CS_NULL);

    Cy_SCB_SPI_Disable(CAMERA_SPI_HW_SCB, NULL);
    Cy_SCB_SPI_DeInit(CAMERA_SPI_HW_SCB);
    memset(&s_camera_spi_hw, 0, sizeof(s_camera_spi_hw));
    s_camera_spi_hw.active_slave = CAMERA_SPI_HW_SLAVE_1;

    camera_spi_hw_build_config(&spi_config);
    (void)Cy_SCB_SPI_Init(CAMERA_SPI_HW_SCB, &spi_config, &s_camera_spi_hw.context);
    Cy_SCB_SPI_SetActiveSlaveSelect(CAMERA_SPI_HW_SCB, 0UL);
    Cy_SCB_SPI_Enable(CAMERA_SPI_HW_SCB);

    spi_irq_config.sysIntSrc = CAMERA_SPI_HW_SCB_IRQ;
    spi_irq_config.intIdx = CAMERA_SPI_HW_CPU_IRQ;
    spi_irq_config.isEnabled = true;
    interrupt_init(&spi_irq_config,
                   camera_spi_hw_irq_handler,
                   CAMERA_SPI_HW_IRQ_PRIORITY);

    for(index = 0U; index < CAMERA_SPI_HW_SLAVE_COUNT; index++)
    {
        gpio_init(s_camera_spi_cs_pins[index], GPO, GPIO_HIGH, GPO_PUSH_PULL);
        gpio_init(s_camera_spi_int_pins[index], GPI, GPIO_LOW, GPI_PULL_DOWN);
    }

    exti_init(P02_4, EXTI_TRIGGER_RISING);
    exti_init(P01_1, EXTI_TRIGGER_RISING);
    exti_init(P19_1, EXTI_TRIGGER_RISING);
}

uint8 camera_spi_hw_start_transfer(camera_spi_hw_slave_id_t id,
                                   uint8 *tx_buffer,
                                   uint8 *rx_buffer,
                                   uint16 length)
{
    cy_en_scb_spi_status_t transfer_ret;

    if((id >= CAMERA_SPI_HW_SLAVE_COUNT) ||
       (tx_buffer == NULL) ||
       (rx_buffer == NULL) ||
       (length == 0U))
    {
        return CAMERA_SPI_HW_TRANSFER_ERROR;
    }

    if(s_camera_spi_hw.busy != 0U)
    {
        return CAMERA_SPI_HW_TRANSFER_BUSY;
    }

    Cy_SCB_SPI_ClearRxFifo(CAMERA_SPI_HW_SCB);
    Cy_SCB_SPI_ClearTxFifo(CAMERA_SPI_HW_SCB);

    s_camera_spi_hw.busy = 1U;
    s_camera_spi_hw.active_slave = id;

    gpio_low(s_camera_spi_cs_pins[id]);
    system_delay_us(CAMERA_SPI_HW_CS_SETUP_DELAY_US);

    transfer_ret = Cy_SCB_SPI_Transfer(CAMERA_SPI_HW_SCB,
                                       tx_buffer,
                                       rx_buffer,
                                       length,
                                       &s_camera_spi_hw.context);
    if(transfer_ret != CY_SCB_SPI_SUCCESS)
    {
        gpio_high(s_camera_spi_cs_pins[id]);
        s_camera_spi_hw.busy = 0U;
        return CAMERA_SPI_HW_TRANSFER_ERROR;
    }

    return CAMERA_SPI_HW_TRANSFER_OK;
}

uint8 camera_spi_hw_transfer_finished(void)
{
    uint32 transfer_status;

    if(s_camera_spi_hw.busy == 0U)
    {
        return 0U;
    }

    transfer_status = Cy_SCB_SPI_GetTransferStatus(CAMERA_SPI_HW_SCB,
                                                   &s_camera_spi_hw.context);
    if((transfer_status & CY_SCB_SPI_TRANSFER_ERR) != 0U)
    {
        return 1U;
    }
    return (uint8)(((transfer_status & CY_SCB_SPI_TRANSFER_ACTIVE) == 0U) ? 1U : 0U);
}

uint8 camera_spi_hw_finish_transfer(void)
{
    uint32 transfer_status;
    uint8 result = CAMERA_SPI_HW_TRANSFER_OK;

    if(s_camera_spi_hw.busy == 0U)
    {
        return CAMERA_SPI_HW_TRANSFER_ERROR;
    }

    transfer_status = Cy_SCB_SPI_GetTransferStatus(CAMERA_SPI_HW_SCB,
                                                   &s_camera_spi_hw.context);
    if((transfer_status & CY_SCB_SPI_TRANSFER_ACTIVE) != 0U)
    {
        return CAMERA_SPI_HW_TRANSFER_BUSY;
    }

    if((transfer_status & CY_SCB_SPI_TRANSFER_ERR) != 0U)
    {
        result = CAMERA_SPI_HW_TRANSFER_ERROR;
    }

    gpio_high(s_camera_spi_cs_pins[s_camera_spi_hw.active_slave]);
    s_camera_spi_hw.busy = 0U;

    return result;
}

void camera_spi_hw_abort_transfer(void)
{
    if(s_camera_spi_hw.busy != 0U)
    {
        Cy_SCB_SPI_AbortTransfer(CAMERA_SPI_HW_SCB, &s_camera_spi_hw.context);
        gpio_high(s_camera_spi_cs_pins[s_camera_spi_hw.active_slave]);
        s_camera_spi_hw.busy = 0U;
    }
}

uint8 camera_spi_hw_get_ready_level(camera_spi_hw_slave_id_t id)
{
    if(id >= CAMERA_SPI_HW_SLAVE_COUNT)
    {
        return 0U;
    }

    return gpio_get_level(s_camera_spi_int_pins[id]);
}
