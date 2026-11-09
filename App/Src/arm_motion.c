#include "arm_motion.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "arm_comm.h"
#include "arm_protocol.h"
#include "arm_servo.h"

static QueueHandle_t s_motion_queue;
static TaskHandle_t s_motion_task;
static volatile bool s_is_moving;
static volatile uint32_t s_stop_epoch;
static volatile uint32_t s_completion_count;
static volatile ArmResult s_last_completion_result;

static int16_t ArmMotion_RoundFloatToInt16(float value)
{
    if (value >= 0.0F)
    {
        value += 0.5F;
    }
    else
    {
        value -= 0.5F;
    }

    return (int16_t)value;
}

static void ArmMotion_PublishResult(const ArmMotionCommand *command,
                                    ArmResult result)
{
    ArmEvent event;

    event.type = (result == ARM_RESULT_OK)
                     ? ARM_EVENT_MOTION_DONE
                     : ARM_EVENT_MOTION_STOPPED;
    event.sequence = command->sequence;
    event.source_command = command->source_command;
    event.response_port = command->response_port;
    event.result = result;
    (void)ArmComm_PublishEvent(&event);
}

static void ArmMotion_Task(void *argument)
{
    ArmMotionCommand command;
    int16_t start_angle_x10[ARM_SERVO_COUNT];
    int16_t interpolated_angle_x10[ARM_SERVO_COUNT];
    TickType_t update_delay_ticks;
    uint32_t step_count;
    uint32_t step;
    uint8_t joint;
    ArmResult result;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(s_motion_queue, &command, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        /*
         * STOP 可能在任务阻塞期间到达。epoch 不一致表示当前命令已经被
         * STOP 作废，即使它恰好在 xQueueReset 之前被取出也不会执行。
         */
        if (command.stop_epoch != s_stop_epoch)
        {
            taskENTER_CRITICAL();
            s_last_completion_result = ARM_RESULT_STOPPED;
            ++s_completion_count;
            taskEXIT_CRITICAL();
            ArmMotion_PublishResult(&command, ARM_RESULT_STOPPED);
            continue;
        }

        s_is_moving = true;
        result = ARM_RESULT_OK;
        ArmServo_GetAngles(start_angle_x10);

        step_count =
            ((uint32_t)command.duration_ms + ARM_MOTION_PERIOD_MS - 1U) /
            ARM_MOTION_PERIOD_MS;
        if (step_count == 0U)
        {
            step_count = 1U;
        }

        update_delay_ticks = pdMS_TO_TICKS(ARM_MOTION_PERIOD_MS);
        if (update_delay_ticks == 0U)
        {
            /* 防止将来降低RTOS tick频率后，毫秒换算结果变成0。 */
            update_delay_ticks = 1U;
        }

        for (step = 1U; step <= step_count; ++step)
        {
            float progress;
            float smooth_progress;

            /*
             * 这里不能用vTaskDelayUntil追赶绝对周期：若运动任务曾被中断
             * 或高优先级任务延迟，已经错过的周期会被连续补跑，多个PWM
             * 目标几乎同时写入，机械臂就会表现为“给了3000ms却突然跳动”。
             *
             * 每一步相对延时可保证相邻姿态命令至少间隔一个舵机周期。
             * 系统偶发繁忙时动作只会略微变长，不会被压缩成危险的突跳。
             */
            vTaskDelay(update_delay_ticks);

            if (command.stop_epoch != s_stop_epoch)
            {
                result = ARM_RESULT_STOPPED;
                break;
            }

            progress = (float)step / (float)step_count;
            if (command.source_command ==
                ARM_CMD_CALIBRATION_TRACE)
            {
                /*
                 * 巡边由连续10mm小段组成。若每段都使用零速起停，
                 * 机械臂会每10mm顿一下；线性进度可让相邻小段以
                 * 近似恒速连续衔接。
                 */
                smooth_progress = progress;
            }
            else
            {
                /*
                 * 普通动作使用三次平滑步进：起点和终点速度均为0，
                 * 减少启动、停止以及大角度移动时的舵机冲击。
                 */
                smooth_progress =
                    (3.0F * progress * progress) -
                    (2.0F * progress * progress * progress);
            }

            for (joint = 0U; joint < ARM_SERVO_COUNT; ++joint)
            {
                float value =
                    (float)start_angle_x10[joint] +
                    ((float)(command.target_angle_x10[joint] -
                             start_angle_x10[joint]) *
                     smooth_progress);
                interpolated_angle_x10[joint] =
                    ArmMotion_RoundFloatToInt16(value);
            }

            if (!ArmServo_SetAngles(interpolated_angle_x10))
            {
                result = ARM_RESULT_HW_ERROR;
                break;
            }
        }

        if ((result == ARM_RESULT_OK) &&
            !ArmServo_SetAngles(command.target_angle_x10))
        {
            result = ARM_RESULT_HW_ERROR;
        }

        s_is_moving = false;
        taskENTER_CRITICAL();
        s_last_completion_result = result;
        ++s_completion_count;
        taskEXIT_CRITICAL();
        ArmMotion_PublishResult(&command, result);
    }
}

bool ArmMotion_RtosInit(void)
{
    BaseType_t task_result;

    s_motion_queue = xQueueCreate(ARM_MOTION_QUEUE_DEPTH,
                                  sizeof(ArmMotionCommand));
    if (s_motion_queue == NULL)
    {
        return false;
    }

    s_completion_count = 0U;
    s_last_completion_result = ARM_RESULT_OK;

    task_result = xTaskCreate(ArmMotion_Task,
                              "ArmMotion",
                              ARM_MOTION_TASK_STACK_WORDS,
                              NULL,
                              ARM_MOTION_TASK_PRIORITY,
                              &s_motion_task);

    return (task_result == pdPASS);
}

ArmResult ArmMotion_Submit(const ArmMotionCommand *command)
{
    ArmMotionCommand queued_command;

    if (command == NULL)
    {
        return ARM_RESULT_BAD_PARAMETER;
    }

    if (!ArmServo_ValidateAngles(command->target_angle_x10))
    {
        return ARM_RESULT_ANGLE_LIMIT;
    }

    if (command->duration_ms > ARM_MOTION_MAX_DURATION_MS)
    {
        return ARM_RESULT_BAD_PARAMETER;
    }

    queued_command = *command;
    taskENTER_CRITICAL();
    queued_command.stop_epoch = s_stop_epoch;
    taskEXIT_CRITICAL();

    if (xQueueSend(s_motion_queue, &queued_command, 0U) != pdPASS)
    {
        return ARM_RESULT_QUEUE_FULL;
    }

    return ARM_RESULT_OK;
}

void ArmMotion_RequestStop(void)
{
    taskENTER_CRITICAL();
    ++s_stop_epoch;
    taskEXIT_CRITICAL();

    if (s_motion_queue != NULL)
    {
        (void)xQueueReset(s_motion_queue);
    }
}

bool ArmMotion_IsMoving(void)
{
    return s_is_moving;
}

bool ArmMotion_IsBusy(void)
{
    if (s_is_moving)
    {
        return true;
    }

    return (s_motion_queue != NULL) &&
           (uxQueueMessagesWaiting(s_motion_queue) > 0U);
}

uint32_t ArmMotion_GetCompletionCount(void)
{
    uint32_t value;

    taskENTER_CRITICAL();
    value = s_completion_count;
    taskEXIT_CRITICAL();
    return value;
}

ArmResult ArmMotion_GetLastCompletionResult(void)
{
    ArmResult result;

    taskENTER_CRITICAL();
    result = s_last_completion_result;
    taskEXIT_CRITICAL();
    return result;
}
