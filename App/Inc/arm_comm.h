#ifndef ARM_COMM_H
#define ARM_COMM_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_types.h"

bool ArmComm_RtosInit(void);
bool ArmComm_PublishEvent(const ArmEvent *event);
bool ArmComm_StartVisionTask1(void);
bool ArmComm_StartVisionTask2(void);
bool ArmComm_StartVisionTask3(void);
bool ArmComm_SendVisionCommand(uint8_t command);
/* 触发K230并只解析/打印坐标，不向机械臂动作队列提交计划。 */
bool ArmComm_StartVisionDebug(uint8_t task_id);
bool ArmComm_SimulateVisionFrame(uint8_t task_id);
bool ArmComm_InjectVisionFrame(uint8_t task_id,
                               const uint8_t *frame,
                               uint16_t length);
void ArmComm_GetVisionProtocolStats(uint32_t *valid_frames,
                                    uint32_t *invalid_frames,
                                    uint8_t *active_task);
uint32_t ArmComm_GetLastValidFrameTick(void);

#endif /* ARM_COMM_H */
