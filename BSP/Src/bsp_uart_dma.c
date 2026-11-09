#include "bsp_uart_dma.h"

#include <string.h>

#include "queue.h"
#include "stm32f4xx_hal.h"

#include "bsp_debug_uart_dma.h"

/*
 * K230 视觉通信接口：
 *   USART1_TX -> PB6 (AF7)
 *   USART1_RX -> PB7 (AF7)
 *   USART1_RX -> DMA2 Stream2 Channel4
 *
 * DMA 使用 Normal 模式。HAL 在检测到空闲线或缓冲区满时调用
 * HAL_UARTEx_RxEventCallback，本驱动把本次数据块复制到 FreeRTOS 队列，
 * 然后立即重新启动下一次 DMA 接收。
 */

static UART_HandleTypeDef s_huart1;
static DMA_HandleTypeDef s_hdma_usart1_rx;
static QueueHandle_t s_rx_queue;
static uint8_t s_dma_rx_buffer[ARM_UART_RX_DMA_BUFFER_SIZE];
static BspUartRxChunk s_isr_chunk;

static volatile uint32_t s_rx_event_count;
static volatile uint32_t s_rx_byte_count;
static volatile uint32_t s_dropped_chunk_count;
static volatile uint32_t s_error_count;
static volatile bool s_restart_required;

static bool BspUartDma_ArmReceive(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(&s_huart1,
                                         s_dma_rx_buffer,
                                         ARM_UART_RX_DMA_BUFFER_SIZE);
    if (status != HAL_OK)
    {
        return false;
    }

    /* 不需要半传输回调，避免连续字节流导致无意义的中断。 */
    __HAL_DMA_DISABLE_IT(&s_hdma_usart1_rx, DMA_IT_HT);
    return true;
}

bool BspUartDma_HardwareInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio);

    s_hdma_usart1_rx.Instance = DMA2_Stream2;
    s_hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    s_hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    s_hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    s_hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    s_hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    s_hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    s_hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    s_hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&s_hdma_usart1_rx) != HAL_OK)
    {
        return false;
    }

    s_huart1.Instance = USART1;
    s_huart1.Init.BaudRate = ARM_UART_BAUDRATE;
    s_huart1.Init.WordLength = UART_WORDLENGTH_8B;
    s_huart1.Init.StopBits = UART_STOPBITS_1;
    s_huart1.Init.Parity = UART_PARITY_NONE;
    s_huart1.Init.Mode = UART_MODE_TX_RX;
    s_huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    __HAL_LINKDMA(&s_huart1, hdmarx, s_hdma_usart1_rx);

    if (HAL_UART_Init(&s_huart1) != HAL_OK)
    {
        return false;
    }

    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, ARM_UART_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, ARM_UART_IRQ_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    return true;
}

bool BspUartDma_RtosInit(void)
{
    s_rx_queue = xQueueCreate(ARM_UART_RX_QUEUE_DEPTH,
                              sizeof(BspUartRxChunk));
    return (s_rx_queue != NULL);
}

bool BspUartDma_Start(void)
{
    if (s_rx_queue == NULL)
    {
        return false;
    }

    s_restart_required = false;
    return BspUartDma_ArmReceive();
}

bool BspUartDma_Receive(BspUartRxChunk *chunk, TickType_t timeout)
{
    if ((chunk == NULL) || (s_rx_queue == NULL))
    {
        return false;
    }

    /*
     * UART 错误回调只设置重启标志，实际 Abort/重启放在任务上下文执行，
     * 避免在中断里执行较长的恢复流程。
     */
    if (s_restart_required)
    {
        (void)HAL_UART_AbortReceive(&s_huart1);
        if (BspUartDma_ArmReceive())
        {
            s_restart_required = false;
        }
    }

    return (xQueueReceive(s_rx_queue, chunk, timeout) == pdPASS);
}

bool BspUartDma_Transmit(const uint8_t *data,
                         uint16_t length,
                         uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    return (HAL_UART_Transmit(&s_huart1,
                              (uint8_t *)data,
                              length,
                              timeout_ms) == HAL_OK);
}

uint32_t BspUartDma_GetDroppedChunkCount(void)
{
    return s_dropped_chunk_count;
}

uint32_t BspUartDma_GetRxEventCount(void)
{
    return s_rx_event_count;
}

uint32_t BspUartDma_GetRxByteCount(void)
{
    return s_rx_byte_count;
}

uint32_t BspUartDma_GetErrorCount(void)
{
    return s_error_count;
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_hdma_usart1_rx);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&s_huart1);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (huart != &s_huart1)
    {
        (void)BspDebugUartDma_HandleRxEvent(huart, size);
        return;
    }

    if (size > ARM_UART_RX_DMA_BUFFER_SIZE)
    {
        size = ARM_UART_RX_DMA_BUFFER_SIZE;
    }

    if ((size > 0U) && (s_rx_queue != NULL))
    {
        ++s_rx_event_count;
        s_rx_byte_count += size;
        s_isr_chunk.length = size;
        (void)memcpy(s_isr_chunk.data, s_dma_rx_buffer, size);

        if (xQueueSendFromISR(s_rx_queue,
                              &s_isr_chunk,
                              &higher_priority_task_woken) != pdPASS)
        {
            ++s_dropped_chunk_count;
        }
    }

    if (!BspUartDma_ArmReceive())
    {
        s_restart_required = true;
        ++s_error_count;
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &s_huart1)
    {
        ++s_error_count;
        s_restart_required = true;
        return;
    }

    (void)BspDebugUartDma_HandleError(huart);
}
