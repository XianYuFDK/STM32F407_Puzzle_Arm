#include "arm_puzzle.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "arm_calibration.h"
#include "arm_comm.h"
#include "arm_config.h"
#include "arm_kinematics.h"
#include "arm_motion.h"
#include "arm_protocol.h"
#include "arm_servo.h"
#include "bsp_magnet.h"
#include "bsp_puzzle_indicator.h"

typedef struct
{
    ArmKinematicsSolution pick_safe;
    ArmKinematicsSolution pick_lift_safe;
    ArmKinematicsSolution pick_lift_mid;
    ArmKinematicsSolution pick_surface;
    ArmKinematicsSolution place_safe;
    ArmKinematicsSolution place_surface;
} ArmPuzzlePreparedPiece;

typedef struct
{
    uint16_t approach_ms;
    uint16_t vertical_ms;
    uint16_t transfer_ms;
    uint16_t pick_lift_direct_ms;
    uint16_t pick_lift_mid_ms;
    uint16_t pick_lift_safe_ms;
    uint16_t home_ms;
} ArmPuzzleMotionProfile;

static const ArmPuzzleMotionProfile s_default_motion_profile =
{
    ARM_PUZZLE_APPROACH_DURATION_MS,
    ARM_PUZZLE_VERTICAL_DURATION_MS,
    ARM_PUZZLE_TRANSFER_DURATION_MS,
    ARM_PUZZLE_PICK_LIFT_DIRECT_DURATION_MS,
    ARM_PUZZLE_PICK_LIFT_MID_DURATION_MS,
    ARM_PUZZLE_PICK_LIFT_SAFE_DURATION_MS,
    ARM_PUZZLE_HOME_DURATION_MS
};

static const ArmPuzzleMotionProfile s_task3_motion_profile =
{
    ARM_PUZZLE_TASK3_APPROACH_DURATION_MS,
    ARM_PUZZLE_TASK3_VERTICAL_DURATION_MS,
    ARM_PUZZLE_TASK3_TRANSFER_DURATION_MS,
    ARM_PUZZLE_TASK3_PICK_LIFT_DIRECT_MS,
    ARM_PUZZLE_TASK3_PICK_LIFT_MID_MS,
    ARM_PUZZLE_TASK3_PICK_LIFT_SAFE_MS,
    ARM_PUZZLE_TASK3_HOME_DURATION_MS
};

static QueueHandle_t s_plan_queue;
static TaskHandle_t s_puzzle_task;
static volatile bool s_busy;
static volatile bool s_stop_requested;
static volatile ArmPuzzleState s_state;
static volatile uint8_t s_sequence;
static volatile uint8_t s_current_piece;
static volatile ArmResult s_last_result;
static bool s_last_signature_valid;
static uint32_t s_last_signature;

#define ARM_PUZZLE_SERVO_WRIST_INDEX        1U

#if ARM_PUZZLE_SURFACE_Z_TABLE_ENABLED
static float ArmPuzzle_Clamp01(float value)
{
    if (value < 0.0F)
    {
        return 0.0F;
    }
    if (value > 1.0F)
    {
        return 1.0F;
    }
    return value;
}

static float ArmPuzzle_Lerp(float a, float b, float t)
{
    return a + ((b - a) * t);
}

static float ArmPuzzle_GetRatio(float value, float start, float end)
{
    if ((end - start) < 0.001F)
    {
        return 0.0F;
    }

    return ArmPuzzle_Clamp01((value - start) / (end - start));
}

static float ArmPuzzle_BilinearZ(float x_ratio,
                                 float y_ratio,
                                 float z00,
                                 float z10,
                                 float z01,
                                 float z11)
{
    float z0 = ArmPuzzle_Lerp(z00, z10, x_ratio);
    float z1 = ArmPuzzle_Lerp(z01, z11, x_ratio);

    return ArmPuzzle_Lerp(z0, z1, y_ratio);
}
#endif

static float ArmPuzzle_GetSurfaceHeight(float x_mm,
                                        float y_mm,
                                        float fallback_height_mm)
{
#if ARM_PUZZLE_SURFACE_Z_TABLE_ENABLED
    if (x_mm <= ARM_PUZZLE_SURFACE_Z_X1_MM)
    {
        float x_ratio =
            ArmPuzzle_GetRatio(x_mm,
                               ARM_PUZZLE_SURFACE_Z_X0_MM,
                               ARM_PUZZLE_SURFACE_Z_X1_MM);

        if (y_mm <= ARM_PUZZLE_SURFACE_Z_Y1_MM)
        {
            float y_ratio =
                ArmPuzzle_GetRatio(y_mm,
                                   ARM_PUZZLE_SURFACE_Z_Y0_MM,
                                   ARM_PUZZLE_SURFACE_Z_Y1_MM);

            return ArmPuzzle_BilinearZ(x_ratio,
                                       y_ratio,
                                       ARM_PUZZLE_SURFACE_Z_00_MM,
                                       ARM_PUZZLE_SURFACE_Z_10_MM,
                                       ARM_PUZZLE_SURFACE_Z_01_MM,
                                       ARM_PUZZLE_SURFACE_Z_11_MM);
        }
        else
        {
            float y_ratio =
                ArmPuzzle_GetRatio(y_mm,
                                   ARM_PUZZLE_SURFACE_Z_Y1_MM,
                                   ARM_PUZZLE_SURFACE_Z_Y2_MM);

            return ArmPuzzle_BilinearZ(x_ratio,
                                       y_ratio,
                                       ARM_PUZZLE_SURFACE_Z_01_MM,
                                       ARM_PUZZLE_SURFACE_Z_11_MM,
                                       ARM_PUZZLE_SURFACE_Z_02_MM,
                                       ARM_PUZZLE_SURFACE_Z_12_MM);
        }
    }
    else
    {
        float x_ratio =
            ArmPuzzle_GetRatio(x_mm,
                               ARM_PUZZLE_SURFACE_Z_X1_MM,
                               ARM_PUZZLE_SURFACE_Z_X2_MM);

        if (y_mm <= ARM_PUZZLE_SURFACE_Z_Y1_MM)
        {
            float y_ratio =
                ArmPuzzle_GetRatio(y_mm,
                                   ARM_PUZZLE_SURFACE_Z_Y0_MM,
                                   ARM_PUZZLE_SURFACE_Z_Y1_MM);

            return ArmPuzzle_BilinearZ(x_ratio,
                                       y_ratio,
                                       ARM_PUZZLE_SURFACE_Z_10_MM,
                                       ARM_PUZZLE_SURFACE_Z_20_MM,
                                       ARM_PUZZLE_SURFACE_Z_11_MM,
                                       ARM_PUZZLE_SURFACE_Z_21_MM);
        }
        else
        {
            float y_ratio =
                ArmPuzzle_GetRatio(y_mm,
                                   ARM_PUZZLE_SURFACE_Z_Y1_MM,
                                   ARM_PUZZLE_SURFACE_Z_Y2_MM);

            return ArmPuzzle_BilinearZ(x_ratio,
                                       y_ratio,
                                       ARM_PUZZLE_SURFACE_Z_11_MM,
                                       ARM_PUZZLE_SURFACE_Z_21_MM,
                                       ARM_PUZZLE_SURFACE_Z_12_MM,
                                       ARM_PUZZLE_SURFACE_Z_22_MM);
        }
    }
#else
    (void)x_mm;
    (void)y_mm;
    return fallback_height_mm;
#endif
}

static float ArmPuzzle_GetPickXHeightComp(float x_mm)
{
#if ARM_PUZZLE_PICK_X_Z_COMP_ENABLED
    float ratio;

    if (x_mm <= ARM_PUZZLE_PICK_X_Z_COMP_NEAR_X_MM)
    {
        return ARM_PUZZLE_PICK_X_Z_COMP_NEAR_MM;
    }
    if (x_mm >= ARM_PUZZLE_PICK_X_Z_COMP_FAR_X_MM)
    {
        return ARM_PUZZLE_PICK_X_Z_COMP_FAR_MM;
    }

    ratio =
        (x_mm - ARM_PUZZLE_PICK_X_Z_COMP_NEAR_X_MM) /
        (ARM_PUZZLE_PICK_X_Z_COMP_FAR_X_MM -
         ARM_PUZZLE_PICK_X_Z_COMP_NEAR_X_MM);
    return ARM_PUZZLE_PICK_X_Z_COMP_NEAR_MM +
           ((ARM_PUZZLE_PICK_X_Z_COMP_FAR_MM -
             ARM_PUZZLE_PICK_X_Z_COMP_NEAR_MM) * ratio);
#else
    (void)x_mm;
    return 0.0F;
#endif
}

static ArmResult ArmPuzzle_PreparePiece(
    const ArmPuzzlePiece *piece,
    ArmPuzzlePreparedPiece *prepared)
{
    float pick_x_mm =
        ((float)piece->pick_x_x10_mm / 10.0F) +
        ARM_PUZZLE_VISION_TO_ARM_X_OFFSET_MM;
    float pick_y_mm =
        ((float)piece->pick_y_x10_mm / 10.0F) +
        ARM_PUZZLE_VISION_TO_ARM_Y_OFFSET_MM;
    float place_x_mm =
        ((float)piece->place_x_x10_mm / 10.0F) +
        ARM_PUZZLE_VISION_TO_ARM_X_OFFSET_MM;
    float place_y_mm =
        ((float)piece->place_y_x10_mm / 10.0F) +
        ARM_PUZZLE_VISION_TO_ARM_Y_OFFSET_MM;
    float rotate_deg = (float)piece->rotate_x100_deg / 100.0F;
    float pick_base_yaw;
    float place_base_yaw;
    float required_relative_rotation;
    float pick_magnet_relative;
    float place_magnet_relative;
    float pick_surface_height_mm;
    float place_surface_height_mm;

    if (!ArmKinematics_GetBaseYawDeg(pick_x_mm,
                                     pick_y_mm,
                                     &pick_base_yaw) ||
        !ArmKinematics_GetBaseYawDeg(place_x_mm,
                                     place_y_mm,
                                     &place_base_yaw))
    {
        return ARM_RESULT_IK_UNREACHABLE;
    }

    /*
     * 视觉给的是铁片在世界坐标中的旋转量。搬运时底座自身也会让
     * 铁片转动(place_yaw-pick_yaw)，故末端旋转舵机只补足剩余角度。
     *
     * J1实物安装存在轻微偏心/端面倾斜时，抓取前旋转J1会改变电磁铁
     * 最低点高度，使同坐标GOTO准确而视觉抓取Z偏差。现在优先让抓取
     * 相对角为0，在吸起到安全高度后的搬运阶段完成旋转。只有总旋转
     * 超出单侧90度范围时，抓取端才承担保证J1不越界的最小必要角度。
     */
    required_relative_rotation =
        ArmKinematics_NormalizeDeg(
            rotate_deg - (place_base_yaw - pick_base_yaw));
    if (required_relative_rotation > 90.0F)
    {
        place_magnet_relative = 90.0F;
        pick_magnet_relative =
            90.0F - required_relative_rotation;
    }
    else if (required_relative_rotation < -90.0F)
    {
        place_magnet_relative = -90.0F;
        pick_magnet_relative =
            -90.0F - required_relative_rotation;
    }
    else
    {
        pick_magnet_relative = 0.0F;
        place_magnet_relative = required_relative_rotation;
    }
    pick_surface_height_mm =
        ArmPuzzle_GetSurfaceHeight(pick_x_mm,
                                   pick_y_mm,
                                   ARM_PUZZLE_PICK_HEIGHT_MM) +
        ARM_PUZZLE_PICK_Z_TRIM_MM +
        ArmPuzzle_GetPickXHeightComp(pick_x_mm);
    place_surface_height_mm =
        ArmPuzzle_GetSurfaceHeight(place_x_mm,
                                   place_y_mm,
                                   ARM_PUZZLE_PLACE_HEIGHT_MM) +
        ARM_PUZZLE_PLACE_Z_TRIM_MM;

    if (!ArmKinematics_SolveBoardPose(
            pick_x_mm,
            pick_y_mm,
            ARM_PUZZLE_TRAVEL_HEIGHT_MM,
            pick_magnet_relative,
            &prepared->pick_safe) ||
        !ArmKinematics_SolveBoardPose(
            pick_x_mm,
            pick_y_mm,
            pick_surface_height_mm,
            pick_magnet_relative,
            &prepared->pick_surface) ||
#if ARM_PUZZLE_PICK_LIFT_SEGMENTED
        !ArmKinematics_SolveBoardPose(
            pick_x_mm,
            pick_y_mm,
            (pick_surface_height_mm + ARM_PUZZLE_TRAVEL_HEIGHT_MM) * 0.5F,
            pick_magnet_relative,
            &prepared->pick_lift_mid) ||
#endif
        !ArmKinematics_SolveBoardPose(
            place_x_mm,
            place_y_mm,
            ARM_PUZZLE_TRAVEL_HEIGHT_MM,
            place_magnet_relative,
            &prepared->place_safe) ||
        !ArmKinematics_SolveBoardPose(
            place_x_mm,
            place_y_mm,
            place_surface_height_mm,
            place_magnet_relative,
            &prepared->place_surface))
    {
        return ARM_RESULT_IK_UNREACHABLE;
    }

    prepared->pick_lift_safe = prepared->pick_safe;
#if !ARM_PUZZLE_PICK_LIFT_SEGMENTED
    prepared->pick_lift_mid = prepared->pick_surface;
#endif

#if ARM_PUZZLE_LOCK_WRIST_AFTER_PICK
    /*
     * 吸合后抬升时不要让腕部舵机继续追随高Z姿态变化。保持抓取瞬间
     * 的J2角度，可以减少铁片被腕部动作带斜、带滑。
     */
    prepared->pick_lift_mid.servo_angle_x10[ARM_PUZZLE_SERVO_WRIST_INDEX] =
        prepared->pick_surface.servo_angle_x10[ARM_PUZZLE_SERVO_WRIST_INDEX];
    prepared->pick_lift_safe.servo_angle_x10[ARM_PUZZLE_SERVO_WRIST_INDEX] =
        prepared->pick_surface.servo_angle_x10[ARM_PUZZLE_SERVO_WRIST_INDEX];
#endif

    return ARM_RESULT_OK;
}

static ArmResult ArmPuzzle_PreparePlan(
    const ArmPuzzlePlan *plan,
    ArmPuzzlePreparedPiece prepared[ARM_PUZZLE_PIECE_COUNT])
{
    uint8_t order;

    if ((plan == NULL) ||
        (plan->count < ARM_PUZZLE_MIN_PIECE_COUNT) ||
        (plan->count > ARM_PUZZLE_PIECE_COUNT))
    {
        return ARM_RESULT_BAD_PARAMETER;
    }

    for (order = 0U; order < plan->count; ++order)
    {
        ArmResult result =
            ArmPuzzle_PreparePiece(&plan->pieces[order],
                                   &prepared[order]);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }
    }

    return ARM_RESULT_OK;
}

static bool ArmPuzzle_TimeExpired(uint32_t start_tick)
{
    return ((HAL_GetTick() - start_tick) >= ARM_PUZZLE_PLAN_TIMEOUT_MS);
}

static ArmResult ArmPuzzle_DelayChecked(uint32_t delay_ms,
                                        uint32_t plan_start_tick)
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
        if (ArmPuzzle_TimeExpired(plan_start_tick))
        {
            return ARM_RESULT_PLAN_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(slice));
        elapsed += slice;
    }

    return ARM_RESULT_OK;
}

static ArmResult ArmPuzzle_Move(
    const ArmKinematicsSolution *solution,
    uint16_t duration_ms,
    uint8_t sequence,
    uint32_t plan_start_tick)
{
    ArmMotionCommand command;
    ArmResult submit_result;
    uint32_t completion_before;
    uint32_t wait_start;
    uint8_t joint;

    if (s_stop_requested)
    {
        return ARM_RESULT_STOPPED;
    }
    if (ArmPuzzle_TimeExpired(plan_start_tick))
    {
        return ARM_RESULT_PLAN_TIMEOUT;
    }

    command.sequence = sequence;
    command.source_command = ARM_CMD_PUZZLE_STEP;
    command.response_port = ARM_COMM_PORT_NONE;
    command.duration_ms = duration_ms;
    command.stop_epoch = 0U;

    for (joint = 0U; joint < ARM_SERVO_COUNT; ++joint)
    {
        command.target_angle_x10[joint] =
            solution->servo_angle_x10[joint];
    }

    completion_before = ArmMotion_GetCompletionCount();
    submit_result = ArmMotion_Submit(&command);
    if (submit_result != ARM_RESULT_OK)
    {
        return submit_result;
    }

    wait_start = HAL_GetTick();
    while (ArmMotion_GetCompletionCount() == completion_before)
    {
        if (s_stop_requested)
        {
            ArmMotion_RequestStop();
        }
        if (ArmPuzzle_TimeExpired(plan_start_tick) ||
            ((HAL_GetTick() - wait_start) >
             ((uint32_t)duration_ms + ARM_PUZZLE_MOTION_TIMEOUT_MARGIN_MS)))
        {
            ArmMotion_RequestStop();
            return ARM_RESULT_PLAN_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(10U));
    }

    return ArmMotion_GetLastCompletionResult();
}

static ArmResult ArmPuzzle_ExecutePrepared(
    const ArmPuzzlePlan *plan,
    const ArmPuzzlePreparedPiece prepared[ARM_PUZZLE_PIECE_COUNT])
{
    const ArmPuzzleMotionProfile *motion_profile =
        (plan->task_id == 3U)
            ? &s_task3_motion_profile
            : &s_default_motion_profile;
    uint32_t plan_start_tick = HAL_GetTick();
    uint8_t order;

    for (order = 0U; order < plan->count; ++order)
    {
        ArmResult result;
        const ArmPuzzlePreparedPiece *piece = &prepared[order];

        s_current_piece = (uint8_t)(order + 1U);

        result = ArmPuzzle_Move(&piece->pick_safe,
                                motion_profile->approach_ms,
                                plan->sequence,
                                plan_start_tick);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        result = ArmPuzzle_Move(&piece->pick_surface,
                                motion_profile->vertical_ms,
                                plan->sequence,
                                plan_start_tick);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        /*
         * 到达抓取高度后先保持姿态，等待长连杆振动和舵机回差稳定。
         * 延时结束后才闭合PD12继电器，防止下降过程中提前吸偏铁片。
         */
        result = ArmPuzzle_DelayChecked(ARM_PUZZLE_PRE_PICK_SETTLE_MS,
                                        plan_start_tick);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        BspMagnet_Set(true);
        result = ArmPuzzle_DelayChecked(ARM_PUZZLE_MAGNET_ON_SETTLE_MS,
                                        plan_start_tick);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

#if ARM_PUZZLE_PICK_LIFT_SEGMENTED
        /*
         * 抓取后抬升仍使用逆解姿态，保证电磁铁吸附面保持平行；
         * 只是拆成两段，让腕部补偿更慢、更连续。
         */
        result = ArmPuzzle_Move(&piece->pick_lift_mid,
                                motion_profile->pick_lift_mid_ms,
                                plan->sequence,
                                plan_start_tick);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        result = ArmPuzzle_Move(&piece->pick_lift_safe,
                                motion_profile->pick_lift_safe_ms,
                                plan->sequence,
                                plan_start_tick);
#else
        /*
         * 不经过中间高度：J2/J3/J4按同一进度联动，一次抬到安全
         * 高度。三个物理关节角之和保持工具俯仰不变，不能锁死J2。
         */
        result = ArmPuzzle_Move(&piece->pick_lift_safe,
                                motion_profile->pick_lift_direct_ms,
                                plan->sequence,
                                plan_start_tick);
#endif
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        /*
         * 抬升到安全高度后，直接从抓取点姿态插补到放置点安全姿态。
         * J1（铁片旋转）与底座、肩、肘、腕同时运动，实现边移动边
         * 旋转；J2/J3/J4仍按两端逆解姿态同步联动，保持吸附面平行。
         */
        result = ArmPuzzle_Move(&piece->place_safe,
                                motion_profile->transfer_ms,
                                plan->sequence,
                                plan_start_tick);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        result = ArmPuzzle_Move(&piece->place_surface,
                                motion_profile->vertical_ms,
                                plan->sequence,
                                plan_start_tick);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        result = ArmPuzzle_DelayChecked(ARM_PUZZLE_PRE_PLACE_SETTLE_MS,
                                        plan_start_tick);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        BspMagnet_Set(false);
        if ((uint8_t)(order + 1U) == plan->count)
        {
            /*
             * 完成标志以最后一块实际放下为准：电磁铁断开后保持当前
             * 放置姿态800ms，确认铁片释放，再拉高PE9触发蜂鸣器。
             * 后续抬升和回HOME不再推迟完成提示。
             */
            result = ArmPuzzle_DelayChecked(ARM_PUZZLE_DONE_DELAY_MS,
                                            plan_start_tick);
            if (result == ARM_RESULT_OK)
            {
                BspPuzzleIndicator_Set(true);
                result = ArmPuzzle_DelayChecked(
                    ARM_PUZZLE_DONE_PULSE_MS,
                    plan_start_tick);
                BspPuzzleIndicator_Set(false);
            }
        }
        else
        {
            result = ArmPuzzle_DelayChecked(
                ARM_PUZZLE_MAGNET_OFF_SETTLE_MS,
                plan_start_tick);
        }
        if (result != ARM_RESULT_OK)
        {
            return result;
        }

        result = ArmPuzzle_Move(&piece->place_safe,
                                motion_profile->vertical_ms,
                                plan->sequence,
                                plan_start_tick);
        if (result != ARM_RESULT_OK)
        {
            return result;
        }
    }

#if ARM_PUZZLE_RETURN_HOME
    {
        ArmKinematicsSolution home_solution;

        ArmServo_GetHomeAngles(home_solution.servo_angle_x10);

        return ArmPuzzle_Move(&home_solution,
                              motion_profile->home_ms,
                              plan->sequence,
                              plan_start_tick);
    }
#else
    return ARM_RESULT_OK;
#endif
}

static void ArmPuzzle_PublishFinal(const ArmPuzzlePlan *plan,
                                   ArmResult result)
{
    ArmEvent event;

    event.type = (result == ARM_RESULT_OK)
                     ? ARM_EVENT_MOTION_DONE
                     : ARM_EVENT_MOTION_STOPPED;
    event.sequence = plan->sequence;
    event.source_command = ARM_CMD_PUZZLE_PLAN;
    event.response_port = ARM_COMM_PORT_DEBUG;
    event.result = result;
    (void)ArmComm_PublishEvent(&event);
}

static void ArmPuzzle_Task(void *argument)
{
    ArmPuzzlePlan plan;
    ArmPuzzlePreparedPiece prepared[ARM_PUZZLE_PIECE_COUNT];

    (void)argument;

    for (;;)
    {
        ArmResult result;

        if (xQueueReceive(s_plan_queue, &plan, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        s_state = ARM_PUZZLE_STATE_RUNNING;
        BspPuzzleIndicator_Set(false);
        s_sequence = plan.sequence;
        s_current_piece = 0U;
        s_last_result = ARM_RESULT_OK;

        /*
         * 在打开电磁铁前预解算整套8个XY点、两种高度和两种末端角。
         * 任一点不可达都整套拒绝，避免拼到中途停在纸面上。
         */
        result = ArmPuzzle_PreparePlan(&plan, prepared);
        if (result == ARM_RESULT_OK)
        {
            result = ArmPuzzle_ExecutePrepared(&plan, prepared);
        }

        BspMagnet_Set(false);
        s_last_result = result;

        if (result == ARM_RESULT_OK)
        {
            s_state = ARM_PUZZLE_STATE_COMPLETE;
        }
        else if (result == ARM_RESULT_STOPPED)
        {
            s_state = ARM_PUZZLE_STATE_STOPPED;
        }
        else
        {
            s_state = ARM_PUZZLE_STATE_ERROR;
        }

        s_current_piece = 0U;
        taskENTER_CRITICAL();
        s_busy = false;
        taskEXIT_CRITICAL();
        ArmPuzzle_PublishFinal(&plan, result);
    }
}

bool ArmPuzzle_RtosInit(void)
{
    BaseType_t task_result;

    s_plan_queue =
        xQueueCreate(ARM_PUZZLE_PLAN_QUEUE_DEPTH, sizeof(ArmPuzzlePlan));
    if (s_plan_queue == NULL)
    {
        return false;
    }

    s_busy = false;
    s_stop_requested = false;
    s_state = ARM_PUZZLE_STATE_IDLE;
    s_sequence = 0U;
    s_current_piece = 0U;
    s_last_result = ARM_RESULT_OK;
    s_last_signature_valid = false;
    s_last_signature = 0U;

    task_result = xTaskCreate(ArmPuzzle_Task,
                              "ArmPuzzle",
                              ARM_PUZZLE_TASK_STACK_WORDS,
                              NULL,
                              ARM_PUZZLE_TASK_PRIORITY,
                              &s_puzzle_task);
    return (task_result == pdPASS);
}

ArmPuzzleSubmitResult ArmPuzzle_SubmitPlan(const ArmPuzzlePlan *plan)
{
    bool old_signature_valid;
    uint32_t old_signature;

    if ((plan == NULL) ||
        (s_plan_queue == NULL) ||
        (plan->count < ARM_PUZZLE_MIN_PIECE_COUNT) ||
        (plan->count > ARM_PUZZLE_PIECE_COUNT))
    {
        return ARM_PUZZLE_SUBMIT_INVALID;
    }

    taskENTER_CRITICAL();
    if (s_last_signature_valid &&
        (plan->signature == s_last_signature))
    {
        taskEXIT_CRITICAL();
        return ARM_PUZZLE_SUBMIT_DUPLICATE;
    }

    if (s_busy || ArmCalibration_IsBusy() || ArmMotion_IsBusy())
    {
        taskEXIT_CRITICAL();
        return ARM_PUZZLE_SUBMIT_BUSY;
    }

    old_signature_valid = s_last_signature_valid;
    old_signature = s_last_signature;
    s_busy = true;
    s_stop_requested = false;
    s_state = ARM_PUZZLE_STATE_QUEUED;
    s_sequence = plan->sequence;
    s_current_piece = 0U;
    s_last_signature_valid = true;
    s_last_signature = plan->signature;
    taskEXIT_CRITICAL();

    if (xQueueSend(s_plan_queue, plan, 0U) != pdPASS)
    {
        taskENTER_CRITICAL();
        s_busy = false;
        s_state = ARM_PUZZLE_STATE_ERROR;
        s_last_result = ARM_RESULT_QUEUE_FULL;
        s_last_signature_valid = old_signature_valid;
        s_last_signature = old_signature;
        taskEXIT_CRITICAL();
        return ARM_PUZZLE_SUBMIT_BUSY;
    }

    return ARM_PUZZLE_SUBMIT_ACCEPTED;
}

void ArmPuzzle_RequestStop(void)
{
    s_stop_requested = true;
    ArmMotion_RequestStop();
    BspMagnet_Set(false);
}

bool ArmPuzzle_IsBusy(void)
{
    return s_busy;
}

void ArmPuzzle_GetStatus(ArmPuzzleStatus *status)
{
    if (status == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    status->state = s_state;
    status->sequence = s_sequence;
    status->current_piece = s_current_piece;
    status->last_result = s_last_result;
    taskEXIT_CRITICAL();
}

const char *ArmPuzzle_StateName(ArmPuzzleState state)
{
    switch (state)
    {
        case ARM_PUZZLE_STATE_IDLE:
            return "IDLE";
        case ARM_PUZZLE_STATE_QUEUED:
            return "QUEUED";
        case ARM_PUZZLE_STATE_RUNNING:
            return "RUNNING";
        case ARM_PUZZLE_STATE_COMPLETE:
            return "COMPLETE";
        case ARM_PUZZLE_STATE_ERROR:
            return "ERROR";
        case ARM_PUZZLE_STATE_STOPPED:
            return "STOPPED";
        default:
            return "UNKNOWN";
    }
}
