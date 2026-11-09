#ifndef BSP_SERVO_PWM_H
#define BSP_SERVO_PWM_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_config.h"

bool BspServoPwm_Init(void);
bool BspServoPwm_Start(void);
bool BspServoPwm_SetPulseUs(uint8_t servo_index, uint16_t pulse_us);

#endif /* BSP_SERVO_PWM_H */
