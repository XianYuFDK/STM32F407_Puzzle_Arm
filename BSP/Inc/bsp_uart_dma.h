#ifndef BSP_UART_DMA_H
#define BSP_UART_DMA_H

#include <stdbool.h>
#include "bsp_uart_common.h"

bool BspUartDma_HardwareInit(void);
bool BspUartDma_RtosInit(void);
bool BspUartDma_Start(void);
bool BspUartDma_Receive(BspUartRxChunk *chunk, TickType_t timeout);
bool BspUartDma_Transmit(const uint8_t *data, uint16_t length, uint32_t timeout_ms);
uint32_t BspUartDma_GetRxEventCount(void);
uint32_t BspUartDma_GetRxByteCount(void);
uint32_t BspUartDma_GetDroppedChunkCount(void);
uint32_t BspUartDma_GetErrorCount(void);

#endif /* BSP_UART_DMA_H */
