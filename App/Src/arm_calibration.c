#include "arm_calibration.h"

#include <math.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "arm_comm.h"
#include "arm_config.h"
#include "arm_motion.h"
#include "arm_protocol.h"
#include "arm_puzzle.h"
#include "arm_servo.h"

typedef struct
{
    uint8_t first_point;
    uint8_t point_count;
    bool return_home;
    bool trace_border;
    ArmKinematicsSolution origin_solution;
    ArmKinematicsSolution solution[ARM_CALIBRATION_POINT_COUNT];
} ArmCalibrationRequest;

typedef struct
{
    float x_mm;
    float y_mm;
} ArmCalibrationPoint;

/*
 * 题面原点为A4左边线与橙色中线交点，X向右、Y向下。
 * 按用户指定顺序从左上角开始，顺时针沿A4外框巡检。
 */
static const ArmCalibrationPoint
    s_points[ARM_CALIBRATION_POINT_COUNT] =
{
    {0.0F, -ARM_CALIBRATION_A4_HALF_HEIGHT_MM}, /* P1 左上 */
    {ARM_CALIBRATION_A4_WIDTH_MM,
     -ARM_CALIBRATION_A4_HALF_HEIGHT_MM},       /* P2 右上 */
    {ARM_CALIBRATION_A4_WIDTH_MM, 0.0F},        /* P3 右中 */
    {ARM_CALIBRATION_A4_WIDTH_MM,
     ARM_CALIBRATION_A4_HALF_HEIGHT_MM},        /* P4 右下 */
    {0.0F, ARM_CALIBRATION_A4_HALF_HEIGHT_MM},  /* P5 左下 */
    {0.0F, 0.0F}                                /* P6 左中 */
};

static QueueHandle_t s_request_queue;
static TaskHandle_t s_calibration_task;
static volatile bool s_busy;
static volatile bool s_stop_requested;
static volatile ArmCalibrationState s_state;
static volatile uint8_t s_current_point;
static volatile uint8_t s_failed_point;
static volatile ArmResult s_last_result;

static ArmResult ArmCalibration_Move(const ArmKinematicsSolution *solution,
                                      uint8_t point_number,
                                      uint16_t duration_ms,
                                      uint8_t source_command);

bool ArmCalibration_GetPoint(uint8_t point_number,
                             float *board_x_mm,
                             float *board_y_mm)
{
    uint8_t index;

    if ((point_number < 1U) ||
        (point_number > ARM_CALIBRATION_POINT_COUNT) ||
        (board_x_mm == NULL) ||
        (board_y_mm == NULL))
    {
        return false;
    }

    index = (uint8_t)(point_number - 1U);
    *board_x_mm = s_points[index].x_mm;
    *board_y_mm = s_points[index].y_mm;
    return true;
}

bool ArmCalibration_SolvePoint(uint8_t point_number,
                               ArmKinematicsSolution *solution)
{
    uint8_t index;

    if ((point_number < 1U) ||
        (point_number > ARM_CALIBRATION_POINT_COUNT) ||
        (solution == NULL))
    {
        return false;
    }

    index = (uint8_t)(point_number - 1U);
    return ArmKinematics_SolveBoardPose(
        s_points[index].x_mm,
        s_points[index].y_mm,
        ARM_CALIBRATION_HEIGHT_MM,
        ARM_CALIBRATION_MAGNET_ANGLE_DEG,
        solution);
}

#if 0
static uint16_t ArmCalibration_GetSegmentCount(
    const ArmCalibrationPoint *start,
    const ArmCalibrationPoint *end)
{
    float dx;
    float dy;
    float distance_mm;
    uint16_t segment_count;

    dx = end->x_mm - start->x_mm;
    dy = end->y_mm - start->y_mm;
    distance_mm = sqrtf((dx * dx) + (dy * dy));
    segment_count =
        (uint16_t)(distance_mm / ARM_CALIBRATION_EDGE_STEP_MM);

    if (((float)segment_count * ARM_CALIBRATION_EDGE_STEP_MM) <
        distance_mm)
    {
        ++segment_count;
    }
    if (segment_count == 0U)
    {
        segment_count = 1U;
    }

    return segment_count;
}

static bool ArmCalibration_CheckBorder(uint8_t *failed_point)
{
    ArmKinematicsSolution solution;
    uint8_t start_index;

    for (start_index = 0U;
         start_index < ARM_CALIBRATION_POINT_COUNT;
         ++start_index)
    {
        uint8_t end_index =
            (uint8_t)((start_index + 1U) %
                      ARM_CALIBRATION_POINT_COUNT);
        uint16_t segment_count =
            ArmCalibration_GetSegmentCount(&s_points[start_index],
                                           &s_points[end_index]);
        uint16_t segment;

        for (segment = 1U; segment <= segment_count; ++segment)
        {
            float ratio =
                (float)segment / (float)segment_count;
            float x_mm =
                s_points[start_index].x_mm +
                ((s_points[end_index].x_mm -
                  s_points[start_index].x_mm) * ratio);
            float y_mm =
                s_points[start_index].y_mm +
                ((s_points[end_index].y_mm -
                  s_points[start_index].y_mm) * ratio);

            if (!ArmKinematics_SolveBoardPose(
                    x_mm,
                    y_mm,
                    ARM_CALIBRATION_HEIGHT_MM,
                    ARM_CALIBRATION_MAGNET_ANGLE_DEG,
                    &solution))
            {
                if (failed_point != NULL)
                {
                    *failed_point = (uint8_t)(end_index + 1U);
                }
                return false;
            }
        }
    }

    if (failed_point != NULL)
    {
        *failed_point = 0U;
    }
    return true;
}
#endif

ArmResult ArmCalibration_Check(uint8_t *reachable_mask)
{
    ArmKinematicsSolution solution;
    uint8_t mask = 0U;
    uint8_t point;

    for (point = 1U; point <= ARM_CALIBRATION_POINT_COUNT; ++point)
    {
        if (ArmCalibration_SolvePoint(point, &solution))
        {
            mask |= (uint8_t)(1U << (point - 1U));
        }
    }

    if (!ArmKinematics_SolveBoardPose(
            0.0F,
            0.0F,
            ARM_CALIBRATION_HEIGHT_MM,
            ARM_CALIBRATION_MAGNET_ANGLE_DEG,
            &solution))
    {
        if (reachable_mask != NULL)
        {
            *reachable_mask = mask;
        }
        return ARM_RESULT_IK_UNREACHABLE;
    }

    if (reachable_mask != NULL)
    {
        *reachable_mask = mask;
    }

    if (mask != ARM_CALIBRATION_ALL_POINTS_MASK)
    {
        return ARM_RESULT_IK_UNREACHABLE;
    }

    return ARM_RESULT_OK;
}

static ArmResult ArmCalibration_PrepareRequest(
    uint8_t first_point,
    uint8_t point_count,
    bool return_home,
    bool trace_border,
    ArmCalibrationRequest *request,
    uint8_t *failed_point)
{
    uint8_t offset;

    if ((request == NULL) ||
        (first_point < 1U) ||
        (point_count == 0U) ||
        (((uint16_t)first_point + (uint16_t)point_count - 1U) >
         ARM_CALIBRATION_POINT_COUNT))
    {
        return ARM_RESULT_BAD_PARAMETER;
    }

    request->first_point = first_point;
    request->point_count = point_count;
    request->return_home = return_home;
    request->trace_border = trace_border;

    if (trace_border &&
        !ArmKinematics_SolveBoardPose(
            0.0F,
            0.0F,
            ARM_CALIBRATION_HEIGHT_MM,
            ARM_CALIBRATION_MAGNET_ANGLE_DEG,
            &request->origin_solution))
    {
        if (failed_point != NULL)
        {
            *failed_point = 0U;
        }
        return ARM_RESULT_IK_UNREACHABLE;
    }

    for (offset = 0U; offset < point_count; ++offset)
    {
        uint8_t point = (uint8_t)(first_point + offset);

        if (!ArmCalibration_SolvePoint(
                point,
                &request->solution[offset]))
        {
            if (failed_point != NULL)
            {
                *failed_point = point;
            }
            return ARM_RESULT_IK_UNREACHABLE;
        }
    }

    if (failed_point != NULL)
    {
        *failed_point = 0U;
    }
    return ARM_RESULT_OK;
}

static ArmResult ArmCalibration_MoveHome(uint8_t source_command)
{
    ArmKinematicsSolution home_solution;

    ArmServo_GetHomeAngles(home_solution.servo_angle_x10);
    s_current_point = 0U;
    return ArmCalibration_Move(&home_solution,
                               0U,
                               ARM_CALIBRATION_HOME_DURATION_MS,
                               source_command);
}

static void ArmCalibration_Publish(uint8_t source_command,
                                   uint8_t sequence,
                                   ArmResult result)
{
    ArmEvent event;

    event.type = (result == ARM_RESULT_OK)
                     ? ARM_EVENT_MOTION_DONE
                     : ARM_EVENT_MOTION_STOPPED;
    event.sequence = sequence;
    event.source_command = source_command;
    event.response_port = ARM_COMM_PORT_DEBUG;
    event.result = result;
    (void)ArmComm_PublishEvent(&event);
}

static ArmResult ArmCalibration_DelayChecked(uint32_t delay_ms)
{
    uint32_t elapsed = 0U;

    while (elapsed < delay_ms)
    {
        uint32_t slice = delay_ms - elapsed;

        if (slice > 10U)
        {
            slice = 10U;
        }
        if (s_stop_requested)
        {
            return ARM_RESULT_STOPPED;
        }

        vTaskDelay(pdMS_TO_TICKS(slice));
        elapsed += slice;
    }

    return ARM_RESULT_OK;
}

static ArmResult ArmCalibration_Move(
    const ArmKinematicsSolution *solution,
    uint8_t point_number,
    uint16_t duration_ms,
    uint8_t source_command)
{
    ArmMotionCommand command;
    ArmResult result;
    uint32_t completion_before;
    uint32_t wait_start;
    uint8_t joint;

    if ((solution == NULL) || s_stop_requested)
    {
        return ARM_RESULT_STOPPED;
    }

    command.sequence = point_number;
    command.source_command = source_command;
    command.response_port = ARM_COMM_PORT_NONE;
    command.duration_ms = duration_ms;
    command.stop_epoch = 0U;

    for (joint = 0U; joint < ARM_SERVO_COUNT; ++joint)
    {
        command.target_angle_x10[joint] =
            solution->servo_angle_x10[joint];
    }

    completion_before = ArmMotion_GetCompletionCount();
    result = ArmMotion_Submit(&command);
    if (result != ARM_RESULT_OK)
    {
        return result;
    }

    wait_start = HAL_GetTick();
    while (ArmMotion_GetCompletionCount() == completion_before)
    {
        if (s_stop_requested)
        {
            ArmMotion_RequestStop();
        }
        if ((HAL_GetTick() - wait_start) >
            ((uint32_t)duration_ms +
             ARM_CALIBRATION_MOTION_TIMEOUT_MARGIN_MS))
        {
            ArmMotion_RequestStop();
            return ARM_RESULT_PLAN_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(10U));
    }

    return ArmMotion_GetLastCompletionResult();
}

#if 0
static uint16_t ArmCalibration_GetEdgeDurationMs(float distance_mm)
{
    float duration_ms =
        (distance_mm * 1000.0F) /
        ARM_CALIBRATION_EDGE_SPEED_MM_S;

    if (duration_ms <
        (float)ARM_CALIBRATION_EDGE_MIN_DURATION_MS)
    {
        duration_ms =
            (float)ARM_CALIBRATION_EDGE_MIN_DURATION_MS;
    }
    if (duration_ms > (float)ARM_MOTION_MAX_DURATION_MS)
    {
        duration_ms = (float)ARM_MOTION_MAX_DURATION_MS;
    }

    return (uint16_t)(duration_ms + 0.5F);
}

static ArmResult ArmCalibration_TraceEdge(uint8_t start_index,
                                          uint8_t end_index)
{
    uint16_t segment_count;
    uint16_t segment;
    float edge_dx;
    float edge_dy;
    float edge_distance_mm;
    float segment_distance_mm;
    uint16_t duration_ms;

    if ((start_index >= ARM_CALIBRATION_POINT_COUNT) ||
        (end_index >= ARM_CALIBRATION_POINT_COUNT))
    {
        return ARM_RESULT_BAD_PARAMETER;
    }

    edge_dx =
        s_points[end_index].x_mm - s_points[start_index].x_mm;
    edge_dy =
        s_points[end_index].y_mm - s_points[start_index].y_mm;
    edge_distance_mm =
        sqrtf((edge_dx * edge_dx) + (edge_dy * edge_dy));
    segment_count =
        ArmCalibration_GetSegmentCount(&s_points[start_index],
                                       &s_points[end_index]);
    segment_distance_mm =
        edge_distance_mm / (float)segment_count;
    duration_ms =
        ArmCalibration_GetEdgeDurationMs(segment_distance_mm);

    for (segment = 1U; segment <= segment_count; ++segment)
    {
        float ratio =
            (float)segment / (float)segment_count;
        float x_mm =
            s_points[start_index].x_mm + (edge_dx * ratio);
        float y_mm =
            s_points[start_index].y_mm + (edge_dy * ratio);
        ArmKinematicsSolution solution;
        ArmResult result;

        if (s_stop_requested)
        {
            return ARM_RESULT_STOPPED;
        }

        if (!ArmKinematics_SolveBoardPose(
                x_mm,
                y_mm,
                ARM_CALIBRATION_HEIGHT_MM,
                ARM_CALIBRATION_MAGNET_ANGLE_DEG,
                &solution))
        {
            return ARM_RESULT_IK_UNREACHABLE;
        }

        s_current_point = (uint8_t)(end_index + 1U);
        result = ArmCalibration_Move(&solution,
                                     s_current_point,
                                     duration_ms,
                                     ARM_CMD_CALIBRATION_TRACE);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }
    }

    return ARM_RESULT_OK;
}

static ArmResult ArmCalibration_ExecuteBorder(
    const ArmCalibrationRequest *request)
{
    ArmKinematicsSolution p1_safe_solution;
    ArmResult result;
    uint8_t start_index;

    /*
     * 先到P1正上方20mm，再竖直下降到5mm巡边高度，避免从HOME直接
     * 插值到低位时扫过纸面。
     */
    if (!ArmKinematics_SolveBoardPose(
            s_points[0].x_mm,
            s_points[0].y_mm,
            ARM_PUZZLE_TRAVEL_HEIGHT_MM,
            ARM_CALIBRATION_MAGNET_ANGLE_DEG,
            &p1_safe_solution))
    {
        return ARM_RESULT_IK_UNREACHABLE;
    }

    s_current_point = 1U;
    result = ArmCalibration_Move(&p1_safe_solution,
                                 1U,
                                 ARM_CALIBRATION_APPROACH_DURATION_MS,
                                 ARM_CMD_CALIBRATION_STEP);
    if (result != ARM_RESULT_OK)
    {
        return result;
    }

    result = ArmCalibration_Move(&request->solution[0],
                                 1U,
                                 ARM_CALIBRATION_VERTICAL_DURATION_MS,
                                 ARM_CMD_CALIBRATION_STEP);
    ArmCalibration_Publish(ARM_CMD_CALIBRATION_STEP, 1U, result);
    if (result != ARM_RESULT_OK)
    {
        return result;
    }

    result = ArmCalibration_DelayChecked(ARM_CALIBRATION_DWELL_MS);
    if (result != ARM_RESULT_OK)
    {
        return result;
    }

    /*
     * P1→P2→P3→P4→P5→P6→P1，最后一段补齐左侧上半边框。
     */
    for (start_index = 0U;
         start_index < ARM_CALIBRATION_POINT_COUNT;
         ++start_index)
    {
        uint8_t end_index =
            (uint8_t)((start_index + 1U) %
                      ARM_CALIBRATION_POINT_COUNT);

        result = ArmCalibration_TraceEdge(start_index, end_index);
        ArmCalibration_Publish(ARM_CMD_CALIBRATION_STEP,
                               (uint8_t)(end_index + 1U),
                               result);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        result = ArmCalibration_DelayChecked(
            ARM_CALIBRATION_DWELL_MS);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }
    }

    /* 闭合回P1后先抬升到20mm，再由通用收尾动作回HOME。 */
    result = ArmCalibration_Move(&p1_safe_solution,
                                 1U,
                                 ARM_CALIBRATION_VERTICAL_DURATION_MS,
                                 ARM_CMD_CALIBRATION_STEP);
    if (result != ARM_RESULT_OK)
    {
        return result;
    }

    return ARM_RESULT_OK;
}
#endif

static ArmResult ArmCalibration_Execute(
    const ArmCalibrationRequest *request)
{
    uint8_t offset;

    if (request->trace_border)
    {
        ArmResult result;

        /*
         * 坐标点检流程：
         *   (0,0) -> HOME -> P1 -> HOME -> P2 -> HOME ... -> P6 -> HOME。
         * 每个点之间都回HOME，方便观察坐标误差，也减少连续运动回差
         * 对下一个测点的影响。
         */
        s_current_point = 0U;
        result = ArmCalibration_Move(&request->origin_solution,
                                     0U,
                                     ARM_CALIBRATION_MOVE_DURATION_MS,
                                     ARM_CMD_CALIBRATION_STEP);
        ArmCalibration_Publish(ARM_CMD_CALIBRATION_STEP, 0U, result);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        result = ArmCalibration_DelayChecked(ARM_CALIBRATION_DWELL_MS);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        result = ArmCalibration_MoveHome(ARM_CMD_CALIBRATION_STEP);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        result = ArmCalibration_DelayChecked(ARM_CALIBRATION_DWELL_MS);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        for (offset = 0U; offset < request->point_count; ++offset)
        {
            uint8_t point =
                (uint8_t)(request->first_point + offset);

            s_current_point = point;
            result = ArmCalibration_Move(
                &request->solution[offset],
                point,
                ARM_CALIBRATION_MOVE_DURATION_MS,
                ARM_CMD_CALIBRATION_STEP);
            ArmCalibration_Publish(ARM_CMD_CALIBRATION_STEP,
                                   point,
                                   result);
            if (result != ARM_RESULT_OK)
            {
                return result;
            }

            result = ArmCalibration_DelayChecked(
                ARM_CALIBRATION_DWELL_MS);
            if (result != ARM_RESULT_OK)
            {
                return result;
            }

            result = ArmCalibration_MoveHome(ARM_CMD_CALIBRATION_STEP);
            if (result != ARM_RESULT_OK)
            {
                return result;
            }

            result = ArmCalibration_DelayChecked(
                ARM_CALIBRATION_DWELL_MS);
            if (result != ARM_RESULT_OK)
            {
                return result;
            }
        }
    }
    else
    {
        for (offset = 0U; offset < request->point_count; ++offset)
        {
            ArmResult result;
            uint8_t point =
                (uint8_t)(request->first_point + offset);

            s_current_point = point;
            result = ArmCalibration_Move(
                &request->solution[offset],
                point,
                ARM_CALIBRATION_MOVE_DURATION_MS,
                ARM_CMD_CALIBRATION_STEP);
            ArmCalibration_Publish(ARM_CMD_CALIBRATION_STEP,
                                   point,
                                   result);
            if (result != ARM_RESULT_OK)
            {
                return result;
            }

            result = ArmCalibration_DelayChecked(
                ARM_CALIBRATION_DWELL_MS);
            if (result != ARM_RESULT_OK)
            {
                return result;
            }
        }

        if (request->return_home)
        {
            ArmResult result = ArmCalibration_MoveHome(
                ARM_CMD_CALIBRATION_STEP);
            if (result != ARM_RESULT_OK)
            {
                return result;
            }
        }
    }

    return ARM_RESULT_OK;
}

static void ArmCalibration_Task(void *argument)
{
    ArmCalibrationRequest request;

    (void)argument;

    for (;;)
    {
        ArmResult result;

        if (xQueueReceive(s_request_queue,
                          &request,
                          portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        s_state = ARM_CALIBRATION_STATE_RUNNING;
        s_current_point = 0U;
        s_last_result = ARM_RESULT_OK;
        result = ArmCalibration_Execute(&request);
        s_last_result = result;

        if (result == ARM_RESULT_OK)
        {
            s_state = ARM_CALIBRATION_STATE_COMPLETE;
        }
        else if (result == ARM_RESULT_STOPPED)
        {
            s_state = ARM_CALIBRATION_STATE_STOPPED;
        }
        else
        {
            s_state = ARM_CALIBRATION_STATE_ERROR;
            s_failed_point = s_current_point;
        }

        s_current_point = 0U;
        taskENTER_CRITICAL();
        s_busy = false;
        taskEXIT_CRITICAL();
        ArmCalibration_Publish(ARM_CMD_CALIBRATION,
                               0U,
                               result);
    }
}

bool ArmCalibration_RtosInit(void)
{
    BaseType_t task_result;

    s_request_queue =
        xQueueCreate(ARM_CALIBRATION_QUEUE_DEPTH,
                     sizeof(ArmCalibrationRequest));
    if (s_request_queue == NULL)
    {
        return false;
    }

    s_busy = false;
    s_stop_requested = false;
    s_state = ARM_CALIBRATION_STATE_IDLE;
    s_current_point = 0U;
    s_failed_point = 0U;
    s_last_result = ARM_RESULT_OK;

    task_result =
        xTaskCreate(ArmCalibration_Task,
                    "ArmCalib",
                    ARM_CALIBRATION_TASK_STACK_WORDS,
                    NULL,
                    ARM_CALIBRATION_TASK_PRIORITY,
                    &s_calibration_task);
    return (task_result == pdPASS);
}

static ArmResult ArmCalibration_Submit(uint8_t first_point,
                                       uint8_t point_count,
                                       bool return_home,
                                       bool trace_border)
{
    ArmCalibrationRequest request;
    ArmResult result;
    uint8_t failed_point = 0U;

    if (s_request_queue == NULL)
    {
        return ARM_RESULT_HW_ERROR;
    }
    if (s_busy || ArmPuzzle_IsBusy() || ArmMotion_IsBusy())
    {
        return ARM_RESULT_BUSY;
    }

    result = ArmCalibration_PrepareRequest(first_point,
                                           point_count,
                                           return_home,
                                           trace_border,
                                           &request,
                                           &failed_point);
    if (result != ARM_RESULT_OK)
    {
        s_failed_point = failed_point;
        s_last_result = result;
        s_state = ARM_CALIBRATION_STATE_ERROR;
        return result;
    }

    taskENTER_CRITICAL();
    s_busy = true;
    s_stop_requested = false;
    s_state = ARM_CALIBRATION_STATE_QUEUED;
    s_current_point = 0U;
    s_failed_point = 0U;
    s_last_result = ARM_RESULT_OK;
    taskEXIT_CRITICAL();

    if (xQueueSend(s_request_queue, &request, 0U) != pdPASS)
    {
        taskENTER_CRITICAL();
        s_busy = false;
        s_state = ARM_CALIBRATION_STATE_ERROR;
        s_last_result = ARM_RESULT_QUEUE_FULL;
        taskEXIT_CRITICAL();
        return ARM_RESULT_QUEUE_FULL;
    }

    return ARM_RESULT_OK;
}

ArmResult ArmCalibration_Start(void)
{
    return ArmCalibration_Submit(
        1U,
        ARM_CALIBRATION_POINT_COUNT,
        true,
        true);
}

ArmResult ArmCalibration_MoveToPoint(uint8_t point_number)
{
    return ArmCalibration_Submit(point_number,
                                 1U,
                                 false,
                                 false);
}

void ArmCalibration_RequestStop(void)
{
    if (!s_busy)
    {
        return;
    }

    s_stop_requested = true;
    ArmMotion_RequestStop();
}

bool ArmCalibration_IsBusy(void)
{
    return s_busy;
}

void ArmCalibration_GetStatus(ArmCalibrationStatus *status)
{
    if (status == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    status->state = s_state;
    status->current_point = s_current_point;
    status->failed_point = s_failed_point;
    status->last_result = s_last_result;
    taskEXIT_CRITICAL();
}

const char *ArmCalibration_StateName(ArmCalibrationState state)
{
    switch (state)
    {
        case ARM_CALIBRATION_STATE_IDLE:
            return "IDLE";
        case ARM_CALIBRATION_STATE_QUEUED:
            return "QUEUED";
        case ARM_CALIBRATION_STATE_RUNNING:
            return "RUNNING";
        case ARM_CALIBRATION_STATE_COMPLETE:
            return "COMPLETE";
        case ARM_CALIBRATION_STATE_ERROR:
            return "ERROR";
        case ARM_CALIBRATION_STATE_STOPPED:
            return "STOPPED";
        default:
            return "UNKNOWN";
    }
}
