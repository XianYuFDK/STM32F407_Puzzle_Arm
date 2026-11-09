#ifndef ARM_SERVO_H
#define ARM_SERVO_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_config.h"

bool ArmServo_Init(void);
bool ArmServo_ValidateAngles(const int16_t angle_x10[ARM_SERVO_COUNT]);
bool ArmServo_SetAngle(uint8_t servo_index, int16_t angle_x10);
bool ArmServo_SetAngles(const int16_t angle_x10[ARM_SERVO_COUNT]);
void ArmServo_GetAngles(int16_t angle_x10[ARM_SERVO_COUNT]);
void ArmServo_GetHomeAngles(int16_t angle_x10[ARM_SERVO_COUNT]);

#endif /* ARM_SERVO_H */
