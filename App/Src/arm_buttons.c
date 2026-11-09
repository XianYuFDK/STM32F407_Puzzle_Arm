#include "arm_buttons.h"

#include "FreeRTOS.h"
#include "task.h"

#include "arm_calibration.h"
#include "arm_comm.h"
#include "arm_config.h"
#include "arm_kinematics.h"
#include "arm_motion.h"
#include "arm_protocol.h"
#include "arm_puzzle.h"
#include "arm_puzzle_protocol.h"
#include "arm_servo.h"
#include "bsp_magnet.h"

#define ARM_BUTTON_COUNT       10U

typedef enum
{
    ARM_BUTTON_ACTION_TASK1 = 0,
    ARM_BUTTON_ACTION_TASK2,
    ARM_BUTTON_ACTION_TASK3,
    ARM_BUTTON_ACTION_GOTO_ORIGIN,
    ARM_BUTTON_ACTION_GOTO_LEFT_MID,
    ARM_BUTTON_ACTION_GOTO_RIGHT_MID,
    ARM_BUTTON_ACTION_HOME,
    ARM_BUTTON_ACTION_STOP,
    ARM_BUTTON_ACTION_VISION_CALIB,
    ARM_BUTTON_ACTION_VISION_AUX_CALIB
} ArmButtonAction;

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    ArmButtonAction action;
} ArmButtonConfig;

typedef struct
{
    bool last_raw_pressed;
    bool stable_pressed;
    uint16_t stable_time_ms;
} ArmButtonState;

static const ArmButtonConfig s_buttons[ARM_BUTTON_COUNT] =
{
    {ARM_BUTTON1_GPIO_PORT, ARM_BUTTON1_GPIO_PIN,
     ARM_BUTTON_ACTION_TASK1},
    {ARM_BUTTON2_GPIO_PORT, ARM_BUTTON2_GPIO_PIN,
     ARM_BUTTON_ACTION_TASK2},
    {ARM_BUTTON3_GPIO_PORT, ARM_BUTTON3_GPIO_PIN,
     ARM_BUTTON_ACTION_TASK3},
    {ARM_BUTTON4_GPIO_PORT, ARM_BUTTON4_GPIO_PIN,
     ARM_BUTTON_ACTION_GOTO_ORIGIN},
    {ARM_BUTTON5_GPIO_PORT, ARM_BUTTON5_GPIO_PIN,
     ARM_BUTTON_ACTION_GOTO_LEFT_MID},
    {ARM_BUTTON6_GPIO_PORT, ARM_BUTTON6_GPIO_PIN,
     ARM_BUTTON_ACTION_GOTO_RIGHT_MID},
    {ARM_BUTTON7_GPIO_PORT, ARM_BUTTON7_GPIO_PIN,
     ARM_BUTTON_ACTION_HOME},
    {ARM_BUTTON8_GPIO_PORT, ARM_BUTTON8_GPIO_PIN,
     ARM_BUTTON_ACTION_STOP},
    {ARM_BUTTON9_GPIO_PORT, ARM_BUTTON9_GPIO_PIN,
     ARM_BUTTON_ACTION_VISION_CALIB},
    {ARM_BUTTON10_GPIO_PORT, ARM_BUTTON10_GPIO_PIN,
     ARM_BUTTON_ACTION_VISION_AUX_CALIB}
};

static ArmButtonState s_button_state[ARM_BUTTON_COUNT];
static TaskHandle_t s_button_task;
static uint8_t s_button_sequence;

static void ArmButtons_EnableGpioClock(GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
#ifdef GPIOF
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
#endif
#ifdef GPIOG
    else if (port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
#endif
#ifdef GPIOH
    else if (port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
#endif
}

static uint8_t ArmButtons_NextSequence(void)
{
    ++s_button_sequence;
    if (s_button_sequence == 0U)
    {
        ++s_button_sequence;
    }
    return s_button_sequence;
}

static bool ArmButtons_IsMotionBusy(void)
{
    return ArmCalibration_IsBusy() ||
           ArmPuzzle_IsBusy() ||
           ArmMotion_IsBusy();
}

static ArmResult ArmButtons_SubmitAngles(
    const int16_t target_angle_x10[ARM_SERVO_COUNT],
    uint16_t duration_ms,
    uint8_t source_command)
{
    ArmMotionCommand command;
    uint8_t joint;

    if (ArmButtons_IsMotionBusy())
    {
        return ARM_RESULT_BUSY;
    }

    command.sequence = ArmButtons_NextSequence();
    command.source_command = source_command;
    command.response_port = ARM_COMM_PORT_NONE;
    command.duration_ms = duration_ms;
    command.stop_epoch = 0U;

    for (joint = 0U; joint < ARM_SERVO_COUNT; ++joint)
    {
        command.target_angle_x10[joint] = target_angle_x10[joint];
    }

    return ArmMotion_Submit(&command);
}

static ArmResult ArmButtons_Goto(float x_mm,
                                 float y_mm,
                                 float z_mm,
                                 float magnet_deg,
                                 uint16_t duration_ms)
{
    ArmKinematicsSolution solution;

    if (!ArmKinematics_SolveBoardPose(x_mm,
                                      y_mm,
                                      z_mm,
                                      magnet_deg,
                                      &solution))
    {
        return ARM_RESULT_IK_UNREACHABLE;
    }

    return ArmButtons_SubmitAngles(solution.servo_angle_x10,
                                   duration_ms,
                                   ARM_CMD_GOTO_BOARD_POSE);
}

static ArmResult ArmButtons_Home(uint16_t duration_ms)
{
    int16_t target_angle_x10[ARM_SERVO_COUNT];

    ArmServo_GetHomeAngles(target_angle_x10);
    return ArmButtons_SubmitAngles(target_angle_x10,
                                   duration_ms,
                                   ARM_CMD_HOME);
}

static void ArmButtons_StopAll(void)
{
    ArmPuzzle_RequestStop();
    ArmCalibration_RequestStop();
    ArmMotion_RequestStop();
    BspMagnet_Set(false);
}

static void ArmButtons_StartVisionTask(uint8_t task_id)
{
    if (ArmButtons_IsMotionBusy())
    {
        return;
    }

    if (task_id == 1U)
    {
        (void)ArmComm_StartVisionTask1();
    }
    else if (task_id == 2U)
    {
        (void)ArmComm_StartVisionTask2();
    }
    else if (task_id == 3U)
    {
        (void)ArmComm_StartVisionTask3();
    }
}

static void ArmButtons_HandleAction(ArmButtonAction action)
{
    switch (action)
    {
        case ARM_BUTTON_ACTION_TASK1:
            ArmButtons_StartVisionTask(1U);
            break;

        case ARM_BUTTON_ACTION_TASK2:
            ArmButtons_StartVisionTask(2U);
            break;

        case ARM_BUTTON_ACTION_TASK3:
            ArmButtons_StartVisionTask(3U);
            break;

        case ARM_BUTTON_ACTION_GOTO_ORIGIN:
            (void)ArmButtons_Goto(0.0F, 0.0F, 5.0F, 0.0F, 2000U);
            break;

        case ARM_BUTTON_ACTION_GOTO_LEFT_MID:
            (void)ArmButtons_Goto(210.0F, 0.0F, 5.0F, 0.0F, 3000U);
            break;

        case ARM_BUTTON_ACTION_GOTO_RIGHT_MID:
            (void)ArmButtons_Goto(105.0F, -70.0F, 1.0F, 0.0F, 3000U);
            break;

        case ARM_BUTTON_ACTION_HOME:
            (void)ArmButtons_Home(3000U);
            break;

        case ARM_BUTTON_ACTION_STOP:
            ArmButtons_StopAll();
            break;

        case ARM_BUTTON_ACTION_VISION_CALIB:
            (void)ArmComm_SendVisionCommand(
                ARM_PUZZLE_VISION_CALIB_TRIGGER);
            break;

        case ARM_BUTTON_ACTION_VISION_AUX_CALIB:
            (void)ArmComm_SendVisionCommand(
                ARM_PUZZLE_VISION_AUX_CALIB_TRIGGER);
            break;

        default:
            break;
    }
}

static bool ArmButtons_ReadPressed(const ArmButtonConfig *button)
{
    if (button == NULL)
    {
        return false;
    }

    return (HAL_GPIO_ReadPin(button->port, button->pin) ==
            ARM_BUTTON_ACTIVE_LEVEL);
}

static void ArmButtons_Task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        uint8_t index;

        for (index = 0U; index < ARM_BUTTON_COUNT; ++index)
        {
            bool raw_pressed = ArmButtons_ReadPressed(&s_buttons[index]);
            ArmButtonState *state = &s_button_state[index];

            if (raw_pressed != state->last_raw_pressed)
            {
                state->last_raw_pressed = raw_pressed;
                state->stable_time_ms = 0U;
                continue;
            }

            if (state->stable_time_ms < ARM_BUTTON_DEBOUNCE_MS)
            {
                state->stable_time_ms =
                    (uint16_t)(state->stable_time_ms +
                               ARM_BUTTON_SCAN_PERIOD_MS);
                continue;
            }

            if (raw_pressed != state->stable_pressed)
            {
                state->stable_pressed = raw_pressed;
                if (raw_pressed)
                {
                    ArmButtons_HandleAction(s_buttons[index].action);
                }
            }
        }

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(ARM_BUTTON_SCAN_PERIOD_MS));
    }
}

bool ArmButtons_HardwareInit(void)
{
#if ARM_BUTTON_ENABLED
    GPIO_InitTypeDef gpio;
    uint8_t index;

    for (index = 0U; index < ARM_BUTTON_COUNT; ++index)
    {
        ArmButtons_EnableGpioClock(s_buttons[index].port);
    }

    for (index = 0U; index < ARM_BUTTON_COUNT; ++index)
    {
        gpio.Pin = s_buttons[index].pin;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = ARM_BUTTON_GPIO_PULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(s_buttons[index].port, &gpio);

        s_button_state[index].last_raw_pressed =
            ArmButtons_ReadPressed(&s_buttons[index]);
        s_button_state[index].stable_pressed =
            s_button_state[index].last_raw_pressed;
        s_button_state[index].stable_time_ms = 0U;
    }

    s_button_sequence = 0U;
#endif
    return true;
}

bool ArmButtons_RtosInit(void)
{
#if ARM_BUTTON_ENABLED
    BaseType_t task_result;

    task_result = xTaskCreate(ArmButtons_Task,
                              "ArmButtons",
                              ARM_BUTTON_TASK_STACK_WORDS,
                              NULL,
                              ARM_BUTTON_TASK_PRIORITY,
                              &s_button_task);
    return (task_result == pdPASS);
#else
    return true;
#endif
}
