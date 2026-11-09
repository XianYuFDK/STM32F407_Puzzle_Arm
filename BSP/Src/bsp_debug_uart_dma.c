#include "bsp_debug_uart_dma.h"

#include <string.h>

#include "queue.h"

/*
 * 电脑上位机调试接口：
 *   USART2_TX -> PA2 (AF7)，连接 USB-TTL 模块 RX
 *   USART2_RX -> PA3 (AF7)，连接 USB-TTL 模块 TX
 *   USART2_RX -> DMA1 Stream5 Channel4
 *
 * USART2 只负责传输电脑调试用的ASCII文本；命令解析在ArmDebugCli模块
 * 和ArmComm任务内完成。USART1仍使用带CRC的二进制视觉协议。
 */

static UART_HandleTypeDef s_huart2;
static DMA_HandleTypeDef s_hdma_usart2_rx;
static QueueHandle_t s_rx_queue;
static uint8_t s_dma_rx_buffer[ARM_UART_RX_DMA_BUFFER_SIZE];
static BspUartRxChunk s_isr_chunk;

static volatile uint32_t s_dropped_chunk_count;
static volatile uint32_t s_error_count;
static volatile bool s_restart_required;

static bool BspDebugUartDma_ArmReceive(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(&s_huart2,
                                         s_dma_rx_buffer,
                                         ARM_UART_RX_DMA_BUFFER_SIZE);
    if (status != HAL_OK)
    {
        return false;
    }

    __HAL_DMA_DISABLE_IT(&s_hdma_usart2_rx, DMA_IT_HT);
    return true;
}

bool BspDebugUartDma_HardwareInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    s_hdma_usart2_rx.Instance = DMA1_Stream5;
    s_hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    s_hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    s_hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    s_hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    s_hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    s_hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    s_hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    s_hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&s_hdma_usart2_rx) != HAL_OK)
    {
        return false;
    }

    s_huart2.Instance = USART2;
    s_huart2.Init.BaudRate = ARM_DEBUG_UART_BAUDRATE;
    s_huart2.Init.WordLength = UART_WORDLENGTH_8B;
    s_huart2.Init.StopBits = UART_STOPBITS_1;
    s_huart2.Init.Parity = UART_PARITY_NONE;
    s_huart2.Init.Mode = UART_MODE_TX_RX;
    s_huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    __HAL_LINKDMA(&s_huart2, hdmarx, s_hdma_usart2_rx);

    if (HAL_UART_Init(&s_huart2) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, ARM_UART_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, ARM_UART_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    return true;
}

bool BspDebugUartDma_RtosInit(void)
{
    s_rx_queue = xQueueCreate(ARM_UART_RX_QUEUE_DEPTH,
                              sizeof(BspUartRxChunk));
    return (s_rx_queue != NULL);
}

bool BspDebugUartDma_Start(void)
{
    if (s_rx_queue == NULL)
    {
        return false;
    }

    s_restart_required = false;
    return BspDebugUartDma_ArmReceive();
}

bool BspDebugUartDma_Receive(BspUartRxChunk *chunk, TickType_t timeout)
{
    if ((chunk == NULL) || (s_rx_queue == NULL))
    {
        return false;
    }

    if (s_restart_required)
    {
        (void)HAL_UART_AbortReceive(&s_huart2);
        if (BspDebugUartDma_ArmReceive())
        {
            s_restart_required = false;
        }
    }

    return (xQueueReceive(s_rx_queue, chunk, timeout) == pdPASS);
}

bool BspDebugUartDma_Transmit(const uint8_t *data,
                              uint16_t length,
                              uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    return (HAL_UART_Transmit(&s_huart2,
                              (uint8_t *)data,
                              length,
                              timeout_ms) == HAL_OK);
}

uint32_t BspDebugUartDma_GetDroppedChunkCount(void)
{
    return s_dropped_chunk_count;
}

uint32_t BspDebugUartDma_GetErrorCount(void)
{
    return s_error_count;
}

void DMA1_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_usart2_rx);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&s_huart2);
}

bool BspDebugUartDma_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (huart != &s_huart2)
    {
        return false;
    }

    if (size > ARM_UART_RX_DMA_BUFFER_SIZE)
    {
        size = ARM_UART_RX_DMA_BUFFER_SIZE;
    }

    if ((size > 0U) && (s_rx_queue != NULL))
    {
        s_isr_chunk.length = size;
        (void)memcpy(s_isr_chunk.data, s_dma_rx_buffer, size);

        if (xQueueSendFromISR(s_rx_queue,
                              &s_isr_chunk,
                              &higher_priority_task_woken) != pdPASS)
        {
            ++s_dropped_chunk_count;
        }
    }

    if (!BspDebugUartDma_ArmReceive())
    {
        s_restart_required = true;
        ++s_error_count;
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
    return true;
}

bool BspDebugUartDma_HandleError(UART_HandleTypeDef *huart)
{
    if (huart != &s_huart2)
    {
        return false;
    }

    ++s_error_count;
    s_restart_required = true;
    return true;
}
