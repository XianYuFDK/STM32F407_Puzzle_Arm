#include "bsp_magnet.h"

#include "arm_config.h"
#include "stm32f4xx_hal.h"

static bool s_is_on;
static uint32_t s_on_since_tick;

static GPIO_PinState BspMagnet_GetInactiveLevel(void)
{
    return (ARM_MAGNET_ACTIVE_LEVEL == GPIO_PIN_SET)
               ? GPIO_PIN_RESET
               : GPIO_PIN_SET;
}

void BspMagnet_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* 先写关闭电平，再切换为输出，防止上电瞬间误吸合。 */
    HAL_GPIO_WritePin(ARM_MAGNET_GPIO_PORT,
                      ARM_MAGNET_GPIO_PIN,
                      BspMagnet_GetInactiveLevel());

    gpio.Pin = ARM_MAGNET_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ARM_MAGNET_GPIO_PORT, &gpio);

    s_is_on = false;
    s_on_since_tick = 0U;
}

void BspMagnet_Set(bool enabled)
{
    HAL_GPIO_WritePin(ARM_MAGNET_GPIO_PORT,
                      ARM_MAGNET_GPIO_PIN,
                      enabled ? ARM_MAGNET_ACTIVE_LEVEL
                              : BspMagnet_GetInactiveLevel());

    if (enabled && !s_is_on)
    {
        s_on_since_tick = HAL_GetTick();
    }

    if (!enabled)
    {
        s_on_since_tick = 0U;
    }

    s_is_on = enabled;
}

bool BspMagnet_IsOn(void)
{
    return s_is_on;
}

uint32_t BspMagnet_GetOnSinceTick(void)
{
    return s_on_since_tick;
}
