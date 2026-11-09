#ifndef BSP_UART_COMMON_H
#define BSP_UART_COMMON_H

#include <stdint.h>

#include "arm_config.h"
#include "FreeRTOS.h"

/*
 * USART1 视觉口和 USART2 调试口共用的数据块类型。
 * DMA 空闲中断只负责把本次收到的字节复制到各自的 FreeRTOS 队列。
 */
typedef struct
{
    uint16_t length;
    uint8_t data[ARM_UART_RX_DMA_BUFFER_SIZE];
} BspUartRxChunk;

#endif /* BSP_UART_COMMON_H */
