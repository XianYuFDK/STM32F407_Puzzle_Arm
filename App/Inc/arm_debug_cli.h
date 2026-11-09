#ifndef ARM_DEBUG_CLI_H
#define ARM_DEBUG_CLI_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_puzzle_protocol.h"
#include "arm_types.h"

/*
 * USART2 文本调试命令行。
 *
 * 调用环境：ArmComm FreeRTOS 任务。
 * 本模块不在串口/DMA中断中解析字符串，也不直接操作DMA接收队列。
 */
void ArmDebugCli_Init(void);
bool ArmDebugCli_HandleChunk(const uint8_t *data, uint16_t length);
void ArmDebugCli_HandleEvent(const ArmEvent *event);
void ArmDebugCli_ReportVisionPlan(const ArmPuzzlePlan *plan);
void ArmDebugCli_ReportVisionReadError(uint8_t task_id,
                                       const char *reason);

#endif /* ARM_DEBUG_CLI_H */
