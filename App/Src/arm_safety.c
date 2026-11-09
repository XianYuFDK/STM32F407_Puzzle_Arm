#include "arm_safety.h"

#include "FreeRTOS.h"
#include "task.h"

#include "arm_calibration.h"
#include "arm_comm.h"
#include "arm_config.h"
#include "arm_motion.h"
#include "arm_puzzle.h"
#include "arm_protocol.h"
#include "arm_types.h"
#include "bsp_magnet.h"
#include "bsp_safety_io.h"

static TaskHandle_t s_safety_task;

static void ArmSafety_Publish(ArmResult result)
{
    ArmEvent event;

    event.type = ARM_EVENT_SAFETY_ERROR;
    event.sequence = 0U;
    event.source_command = ARM_CMD_STOP;
    event.response_port = ARM_COMM_PORT_DEBUG;
    event.result = result;
    (void)ArmComm_PublishEvent(&event);
}

static void ArmSafety_Task(void *argument)
{
    bool estop_latched = false;
    bool comm_timeout_latched = false;
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        bool estop_active = BspSafetyIo_IsEmergencyStopActive();

        if (estop_active && !estop_latched)
        {
            ArmPuzzle_RequestStop();
            ArmCalibration_RequestStop();
            ArmSafety_Publish(ARM_RESULT_ESTOP);
            estop_latched = true;
        }
        else if (!estop_active)
        {
            estop_latched = false;
        }

#if (ARM_MAGNET_MAX_ON_TIME_MS > 0U)
        if (BspMagnet_IsOn() &&
            ((now - BspMagnet_GetOnSinceTick()) >=
             ARM_MAGNET_MAX_ON_TIME_MS))
        {
            ArmPuzzle_RequestStop();
            ArmCalibration_RequestStop();
            ArmSafety_Publish(ARM_RESULT_MAGNET_TIMEOUT);
        }
#endif

#if ARM_COMM_WATCHDOG_ENABLED
        if ((now - ArmComm_GetLastValidFrameTick()) >
            ARM_COMM_WATCHDOG_TIMEOUT_MS)
        {
            if (!comm_timeout_latched)
            {
                ArmPuzzle_RequestStop();
                ArmCalibration_RequestStop();
                ArmSafety_Publish(ARM_RESULT_COMM_TIMEOUT);
                comm_timeout_latched = true;
            }
        }
        else
        {
            comm_timeout_latched = false;
        }
#else
        (void)comm_timeout_latched;
#endif

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ARM_MOTION_PERIOD_MS));
    }
}

bool ArmSafety_RtosInit(void)
{
    BaseType_t task_result;

    task_result = xTaskCreate(ArmSafety_Task,
                              "ArmSafety",
                              ARM_SAFETY_TASK_STACK_WORDS,
                              NULL,
                              ARM_SAFETY_TASK_PRIORITY,
                              &s_safety_task);
    return (task_result == pdPASS);
}
