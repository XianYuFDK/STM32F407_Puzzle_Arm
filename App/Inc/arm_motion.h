#ifndef ARM_MOTION_H
#define ARM_MOTION_H

#include <stdbool.h>

#include "arm_types.h"

bool ArmMotion_RtosInit(void);
ArmResult ArmMotion_Submit(const ArmMotionCommand *command);
void ArmMotion_RequestStop(void);
bool ArmMotion_IsMoving(void);
bool ArmMotion_IsBusy(void);
uint32_t ArmMotion_GetCompletionCount(void);
ArmResult ArmMotion_GetLastCompletionResult(void);

#endif /* ARM_MOTION_H */
