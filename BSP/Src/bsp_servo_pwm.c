#include "bsp_servo_pwm.h"

#include "stm32f4xx_hal.h"

/*
 * PWM 引脚分配：
 *   舵机1（电磁铁旋转，原2号） -> TIM1_CH2 / PE11
 *   舵机2（原3号）             -> TIM1_CH3 / PE13
 *   舵机3（原4号）             -> TIM1_CH4 / PE14
 *   舵机4（原5号）             -> TIM8_CH1 / PC6
 *   舵机5（原6号）             -> TIM8_CH2 / PC7
 *
 * 原夹爪舵机 TIM1_CH1 / PE9 已停用，不输出 PWM。
 *
 * TIM1/TIM8 均位于 APB2。当前系统时钟下 APB2 定时器时钟是 168 MHz。
 * 驱动会根据 RCC 运行时配置计算预分频，得到 1 us 的计数分辨率。
 */

static TIM_HandleTypeDef s_htim1;
static TIM_HandleTypeDef s_htim8;

static uint32_t BspServoPwm_GetApb2TimerClock(void)
{
    uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();

    /* APB 分频不为 1 时，STM32F4 定时器时钟自动乘 2。 */
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_HCLK_DIV1)
    {
        timer_clock *= 2U;
    }

    return timer_clock;
}

static bool BspServoPwm_InitTimer(TIM_HandleTypeDef *timer,
                                  TIM_TypeDef *instance,
                                  uint8_t channel_count)
{
    TIM_OC_InitTypeDef output_compare = {0};
    uint32_t timer_clock = BspServoPwm_GetApb2TimerClock();
    uint32_t prescaler;
    uint32_t channel;

    if ((timer_clock < ARM_SERVO_PWM_TIMER_TICK_HZ) ||
        ((timer_clock % ARM_SERVO_PWM_TIMER_TICK_HZ) != 0U))
    {
        return false;
    }

    prescaler = (timer_clock / ARM_SERVO_PWM_TIMER_TICK_HZ) - 1U;

    timer->Instance = instance;
    timer->Init.Prescaler = prescaler;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period =
        (ARM_SERVO_PWM_TIMER_TICK_HZ / ARM_SERVO_PWM_FREQUENCY_HZ) - 1U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.RepetitionCounter = 0U;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(timer) != HAL_OK)
    {
        return false;
    }

    output_compare.OCMode = TIM_OCMODE_PWM1;
    output_compare.Pulse = 1500U;
    output_compare.OCPolarity = TIM_OCPOLARITY_HIGH;
    output_compare.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    output_compare.OCFastMode = TIM_OCFAST_DISABLE;
    output_compare.OCIdleState = TIM_OCIDLESTATE_RESET;
    output_compare.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    for (channel = 0U; channel < channel_count; ++channel)
    {
        const uint32_t hal_channel[] =
        {
            TIM_CHANNEL_1,
            TIM_CHANNEL_2,
            TIM_CHANNEL_3,
            TIM_CHANNEL_4
        };

        if (HAL_TIM_PWM_ConfigChannel(timer, &output_compare,
                                     hal_channel[channel]) != HAL_OK)
        {
            return false;
        }
    }

    return true;
}

bool BspServoPwm_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &gpio);

    if (!BspServoPwm_InitTimer(&s_htim1, TIM1, 4U))
    {
        return false;
    }

    if (!BspServoPwm_InitTimer(&s_htim8, TIM8, 2U))
    {
        return false;
    }

    return true;
}

bool BspServoPwm_Start(void)
{
    if (HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_2) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_3) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_Start(&s_htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_Start(&s_htim8, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }
    if (HAL_TIM_PWM_Start(&s_htim8, TIM_CHANNEL_2) != HAL_OK)
    {
        return false;
    }

    return true;
}

bool BspServoPwm_SetPulseUs(uint8_t servo_index, uint16_t pulse_us)
{
    TIM_HandleTypeDef *timer;
    uint32_t channel;

    if ((servo_index >= ARM_SERVO_COUNT) ||
        (pulse_us >= (ARM_SERVO_PWM_TIMER_TICK_HZ /
                      ARM_SERVO_PWM_FREQUENCY_HZ)))
    {
        return false;
    }

    switch (servo_index)
    {
        case 0U:
            timer = &s_htim1;
            channel = TIM_CHANNEL_2;
            break;
        case 1U:
            timer = &s_htim1;
            channel = TIM_CHANNEL_3;
            break;
        case 2U:
            timer = &s_htim1;
            channel = TIM_CHANNEL_4;
            break;
        case 3U:
            timer = &s_htim8;
            channel = TIM_CHANNEL_1;
            break;
        case 4U:
            timer = &s_htim8;
            channel = TIM_CHANNEL_2;
            break;
        default:
            return false;
    }

    __HAL_TIM_SET_COMPARE(timer, channel, pulse_us);
    return true;
}
