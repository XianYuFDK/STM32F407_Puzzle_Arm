#include "bsp_safety_io.h"

#include "arm_config.h"
#include "stm32f4xx_hal.h"

void BspSafetyIo_Init(void)
{
#if ARM_ESTOP_ENABLED
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    gpio.Pin = ARM_ESTOP_GPIO_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ARM_ESTOP_GPIO_PORT, &gpio);
#endif
}

bool BspSafetyIo_IsEmergencyStopActive(void)
{
#if ARM_ESTOP_ENABLED
    return (HAL_GPIO_ReadPin(ARM_ESTOP_GPIO_PORT,
                            ARM_ESTOP_GPIO_PIN) == ARM_ESTOP_ACTIVE_LEVEL);
#else
    return false;
#endif
}
