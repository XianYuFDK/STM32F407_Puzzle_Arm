#include "bsp_puzzle_indicator.h"

#include "arm_config.h"

static GPIO_PinState BspPuzzleIndicator_InactiveLevel(void)
{
    return (ARM_PUZZLE_DONE_ACTIVE_LEVEL == GPIO_PIN_SET)
               ? GPIO_PIN_RESET
               : GPIO_PIN_SET;
}

void BspPuzzleIndicator_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    HAL_GPIO_WritePin(ARM_PUZZLE_DONE_GPIO_PORT,
                      ARM_PUZZLE_DONE_GPIO_PIN,
                      BspPuzzleIndicator_InactiveLevel());

    gpio.Pin = ARM_PUZZLE_DONE_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ARM_PUZZLE_DONE_GPIO_PORT, &gpio);
}

void BspPuzzleIndicator_Set(bool enabled)
{
    HAL_GPIO_WritePin(
        ARM_PUZZLE_DONE_GPIO_PORT,
        ARM_PUZZLE_DONE_GPIO_PIN,
        enabled ? ARM_PUZZLE_DONE_ACTIVE_LEVEL
                : BspPuzzleIndicator_InactiveLevel());
}
