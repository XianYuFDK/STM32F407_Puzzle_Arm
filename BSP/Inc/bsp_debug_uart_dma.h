#ifndef BSP_DEBUG_UART_DMA_H
#define BSP_DEBUG_UART_DMA_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_uart_common.h"
#include "stm32f4xx_hal.h"

bool BspDebugUartDma_HardwareInit(void);
bool BspDebugUartDma_RtosInit(void);
bool BspDebugUartDma_Start(void);
bool BspDebugUartDma_Receive(BspUartRxChunk *chunk, TickType_t timeout);
bool BspDebugUartDma_Transmit(const uint8_t *data,
                              uint16_t length,
                              uint32_t timeout_ms);
uint32_t BspDebugUartDma_GetDroppedChunkCount(void);
uint32_t BspDebugUartDma_GetErrorCount(void);

/*
 * 由统一的 HAL UART 回调分发器调用。应用层不应直接调用这两个函数。
 */
bool BspDebugUartDma_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t size);
bool BspDebugUartDma_HandleError(UART_HandleTypeDef *huart);

#endif /* BSP_DEBUG_UART_DMA_H */
