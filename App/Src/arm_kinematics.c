#include "arm_kinematics.h"

#include <math.h>

#include "arm_servo.h"

#define ARM_KIN_PI                         3.14159265358979323846F
#define ARM_KIN_RAD_TO_DEG                 (180.0F / ARM_KIN_PI)
#define ARM_KIN_DEG_TO_RAD                 (ARM_KIN_PI / 180.0F)
#define ARM_KIN_EPSILON                    0.0001F

enum
{
    ARM_KIN_SERVO_MAGNET = 0,
    ARM_KIN_SERVO_WRIST = 1,
    ARM_KIN_SERVO_ELBOW = 2,
    ARM_KIN_SERVO_SHOULDER = 3,
    ARM_KIN_SERVO_BASE = 4
};

#if ARM_KIN_X_AXIS_CALIBRATION_ENABLED
static const float
    s_x_axis_command_mm[ARM_KIN_X_AXIS_CALIBRATION_POINT_COUNT] =
        ARM_KIN_X_AXIS_COMMAND_MM;
static const float
    s_x_axis_measured_mm[ARM_KIN_X_AXIS_CALIBRATION_POINT_COUNT] =
        ARM_KIN_X_AXIS_MEASURED_MM;
#endif

#if ARM_KIN_Y_AXIS_CALIBRATION_ENABLED
static const float
    s_y_axis_command_abs_mm[ARM_KIN_Y_AXIS_CALIBRATION_POINT_COUNT] =
        ARM_KIN_Y_AXIS_COMMAND_ABS_MM;
static const float
    s_y_axis_measured_abs_mm[ARM_KIN_Y_AXIS_CALIBRATION_POINT_COUNT] =
        ARM_KIN_Y_AXIS_MEASURED_ABS_MM;
#endif

#if ARM_KIN_XY_RESIDUAL_ENABLED
static const float s_xy_grid_command_x_mm[ARM_KIN_XY_GRID_X_COUNT] =
    ARM_KIN_XY_GRID_COMMAND_X_MM;
static const float s_xy_grid_command_y_mm[ARM_KIN_XY_GRID_Y_COUNT] =
    ARM_KIN_XY_GRID_COMMAND_Y_MM;
static const float
    s_xy_grid_measured_x_mm[ARM_KIN_XY_GRID_Y_COUNT]
                             [ARM_KIN_XY_GRID_X_COUNT] =
        ARM_KIN_XY_GRID_MEASURED_X_MM;
static const float
    s_xy_grid_measured_y_mm[ARM_KIN_XY_GRID_Y_COUNT]
                             [ARM_KIN_XY_GRID_X_COUNT] =
        ARM_KIN_XY_GRID_MEASURED_Y_MM;
#endif

#if ARM_KIN_X_Z_COMP_ENABLED
static const float s_x_z_comp_x_mm[ARM_KIN_X_Z_COMP_POINT_COUNT] =
    ARM_KIN_X_Z_COMP_X_MM;
static const float s_x_z_comp_z_mm[ARM_KIN_X_Z_COMP_POINT_COUNT] =
    ARM_KIN_X_Z_COMP_Z_MM;
#endif

#if ARM_KIN_BOARD_CORRECTION_ENABLED
/*
 * 行顺序必须与A4六点定义一致：
 * P1左上、P2右上、P3右中、P4右下、P5左下、P6左中。
 */
static const int16_t
    s_board_correction_x10[ARM_CALIBRATION_POINT_COUNT][ARM_SERVO_COUNT] =
{
    ARM_KIN_CORRECTION_P1_X10,
    ARM_KIN_CORRECTION_P2_X10,
    ARM_KIN_CORRECTION_P3_X10,
    ARM_KIN_CORRECTION_P4_X10,
    ARM_KIN_CORRECTION_P5_X10,
    ARM_KIN_CORRECTION_P6_X10
};
#endif

static int16_t ArmKinematics_RoundToI16(float value)
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

static int16_t ArmKinematics_MapJoint(float joint_deg,
                                      int16_t zero_angle_x10,
                                      int8_t direction)
{
    float logical_x10 =
        (float)zero_angle_x10 +
        ((float)direction * joint_deg * 10.0F);

    return ArmKinematics_RoundToI16(logical_x10);
}

static int16_t ArmKinematics_MapJointScaled(float joint_deg,
                                            int16_t zero_angle_x10,
                                            int8_t direction,
                                            float gain)
{
    float logical_x10 =
        (float)zero_angle_x10 +
        ((float)direction * joint_deg * gain * 10.0F);

    return ArmKinematics_RoundToI16(logical_x10);
}

static float ArmKinematics_GetFarPointPitchCompensation(
    float base_radius_mm)
{
#if ARM_KIN_FAR_POINT_PITCH_COMP_ENABLED
    float distance_ratio;

    if ((base_radius_mm <= ARM_KIN_FAR_POINT_PITCH_START_MM) ||
        (ARM_KIN_FAR_POINT_PITCH_MAX_DEG == 0.0F))
    {
        return 0.0F;
    }

    if (base_radius_mm >= ARM_KIN_FAR_POINT_PITCH_FULL_MM)
    {
        distance_ratio = 1.0F;
    }
    else
    {
        distance_ratio =
            (base_radius_mm - ARM_KIN_FAR_POINT_PITCH_START_MM) /
            (ARM_KIN_FAR_POINT_PITCH_FULL_MM -
             ARM_KIN_FAR_POINT_PITCH_START_MM);
    }

    return ARM_KIN_FAR_POINT_PITCH_MAX_DEG * distance_ratio;
#else
    (void)base_radius_mm;
    return 0.0F;
#endif
}

static float ArmKinematics_GetXZCompensation(float board_x_mm)
{
#if ARM_KIN_X_Z_COMP_ENABLED
    uint8_t index;

    if (board_x_mm <= s_x_z_comp_x_mm[0])
    {
        return s_x_z_comp_z_mm[0];
    }

    for (index = 1U; index < ARM_KIN_X_Z_COMP_POINT_COUNT; ++index)
    {
        float x_start = s_x_z_comp_x_mm[index - 1U];
        float x_end = s_x_z_comp_x_mm[index];
        float x_span = x_end - x_start;

        if (x_span <= ARM_KIN_EPSILON)
        {
            continue;
        }

        if (board_x_mm <= x_end)
        {
            float ratio = (board_x_mm - x_start) / x_span;

            return
                s_x_z_comp_z_mm[index - 1U] +
                ((s_x_z_comp_z_mm[index] -
                  s_x_z_comp_z_mm[index - 1U]) * ratio);
        }
    }

    return s_x_z_comp_z_mm[ARM_KIN_X_Z_COMP_POINT_COUNT - 1U];
#else
    (void)board_x_mm;
    return 0.0F;
#endif
}

static float ArmKinematics_GetLocalZCompensation(float board_x_mm,
                                                 float board_y_mm)
{
#if ARM_KIN_LOCAL_Z_COMP_ENABLED
    float dx = fabsf(board_x_mm - ARM_KIN_LOCAL_Z_COMP_CENTER_X_MM);
    float dy = fabsf(board_y_mm - ARM_KIN_LOCAL_Z_COMP_CENTER_Y_MM);
    float x_ratio;
    float y_ratio;

    if ((dx >= ARM_KIN_LOCAL_Z_COMP_HALF_X_MM) ||
        (dy >= ARM_KIN_LOCAL_Z_COMP_HALF_Y_MM) ||
        (ARM_KIN_LOCAL_Z_COMP_Z_MM == 0.0F))
    {
        return 0.0F;
    }

    x_ratio = 1.0F - (dx / ARM_KIN_LOCAL_Z_COMP_HALF_X_MM);
    y_ratio = 1.0F - (dy / ARM_KIN_LOCAL_Z_COMP_HALF_Y_MM);

    return ARM_KIN_LOCAL_Z_COMP_Z_MM * x_ratio * y_ratio;
#else
    (void)board_x_mm;
    (void)board_y_mm;
    return 0.0F;
#endif
}

static float ArmKinematics_GetNegYZCompensation(float board_y_mm)
{
#if ARM_KIN_NEG_Y_Z_COMP_ENABLED
    float ratio;

    if ((board_y_mm >= ARM_KIN_NEG_Y_Z_COMP_START_Y_MM) ||
        (ARM_KIN_NEG_Y_Z_COMP_Z_MM == 0.0F))
    {
        return 0.0F;
    }

    if (board_y_mm <= ARM_KIN_NEG_Y_Z_COMP_FULL_Y_MM)
    {
        ratio = 1.0F;
    }
    else
    {
        ratio =
            (ARM_KIN_NEG_Y_Z_COMP_START_Y_MM - board_y_mm) /
            (ARM_KIN_NEG_Y_Z_COMP_START_Y_MM -
             ARM_KIN_NEG_Y_Z_COMP_FULL_Y_MM);
    }

    return ARM_KIN_NEG_Y_Z_COMP_Z_MM * ratio;
#else
    (void)board_y_mm;
    return 0.0F;
#endif
}

static void ArmKinematics_ApplyBoardCorrection(
    float board_x_mm,
    float board_y_mm,
    int16_t angles[ARM_SERVO_COUNT])
{
#if ARM_KIN_BOARD_CORRECTION_ENABLED
    uint8_t joint;
    uint8_t left_start;
    uint8_t left_end;
    uint8_t right_start;
    uint8_t right_end;
    int16_t correction_x10[ARM_SERVO_COUNT];
    float x_ratio;
    float y_ratio;

    if (board_x_mm <= 0.0F)
    {
        x_ratio = 0.0F;
    }
    else if (board_x_mm >= ARM_CALIBRATION_A4_WIDTH_MM)
    {
        x_ratio = 1.0F;
    }
    else
    {
        x_ratio = board_x_mm / ARM_CALIBRATION_A4_WIDTH_MM;
    }

    if (board_y_mm <= 0.0F)
    {
        left_start = 0U;  /* P1 */
        left_end = 5U;    /* P6 */
        right_start = 1U; /* P2 */
        right_end = 2U;   /* P3 */
        y_ratio =
            (board_y_mm + ARM_CALIBRATION_A4_HALF_HEIGHT_MM) /
            ARM_CALIBRATION_A4_HALF_HEIGHT_MM;
    }
    else
    {
        left_start = 5U;  /* P6 */
        left_end = 4U;    /* P5 */
        right_start = 2U; /* P3 */
        right_end = 3U;   /* P4 */
        y_ratio =
            board_y_mm / ARM_CALIBRATION_A4_HALF_HEIGHT_MM;
    }

    if (y_ratio < 0.0F)
    {
        y_ratio = 0.0F;
    }
    else if (y_ratio > 1.0F)
    {
        y_ratio = 1.0F;
    }

    for (joint = 0U; joint < ARM_SERVO_COUNT; ++joint)
    {
        float left_correction =
            (float)s_board_correction_x10[left_start][joint] +
            ((float)(s_board_correction_x10[left_end][joint] -
                     s_board_correction_x10[left_start][joint]) *
             y_ratio);
        float right_correction =
            (float)s_board_correction_x10[right_start][joint] +
            ((float)(s_board_correction_x10[right_end][joint] -
                     s_board_correction_x10[right_start][joint]) *
             y_ratio);
        float correction =
            left_correction +
            ((right_correction - left_correction) * x_ratio);

        correction_x10[joint] =
            ArmKinematics_RoundToI16(correction);
    }

    /*
     * 双线性插值和逐关节四舍五入可能产生0.1~0.2°和差误差。
     * 用肘、肩补偿反算腕部补偿，严格保持末端绝对俯仰不变。
     */
    correction_x10[ARM_KIN_SERVO_WRIST] =
        (int16_t)(-correction_x10[ARM_KIN_SERVO_ELBOW] -
                  correction_x10[ARM_KIN_SERVO_SHOULDER]);

    for (joint = 0U; joint < ARM_SERVO_COUNT; ++joint)
    {
        angles[joint] =
            (int16_t)(angles[joint] + correction_x10[joint]);
    }
#else
    (void)board_x_mm;
    (void)board_y_mm;
    (void)angles;
#endif
}

float ArmKinematics_NormalizeDeg(float angle_deg)
{
    while (angle_deg > 180.0F)
    {
        angle_deg -= 360.0F;
    }
    while (angle_deg <= -180.0F)
    {
        angle_deg += 360.0F;
    }
    return angle_deg;
}

/*
 * 将用户/视觉期望的实际X坐标反算成标定时应发送的X。表中MEASURED
 * 必须单调不减；相邻实测值相同表示机械死区，该段跳过，避免除以0。
 */
static float ArmKinematics_MapDesiredXToCommandX(float desired_x_mm)
{
#if ARM_KIN_X_AXIS_CALIBRATION_ENABLED
    uint8_t index;

    if (desired_x_mm <= s_x_axis_measured_mm[0])
    {
        return desired_x_mm;
    }

    for (index = 1U;
         index < ARM_KIN_X_AXIS_CALIBRATION_POINT_COUNT;
         ++index)
    {
        float measured_start = s_x_axis_measured_mm[index - 1U];
        float measured_end = s_x_axis_measured_mm[index];
        float measured_span = measured_end - measured_start;

        if (measured_span <= ARM_KIN_EPSILON)
        {
            continue;
        }

        if (desired_x_mm <= measured_end)
        {
            float ratio =
                (desired_x_mm - measured_start) / measured_span;

            return
                s_x_axis_command_mm[index - 1U] +
                ((s_x_axis_command_mm[index] -
                  s_x_axis_command_mm[index - 1U]) * ratio);
        }
    }

    /*
     * 超过标定最大实测位置时不外推。末端已有死区，继续增加命令只会
     * 增大撞限位风险；A4正常坐标0~210mm不会进入这里。
     */
    return
        s_x_axis_command_mm[
            ARM_KIN_X_AXIS_CALIBRATION_POINT_COUNT - 1U];
#else
    return desired_x_mm;
#endif
}

/*
 * Y轴使用|Y|反插值并恢复原符号，使同一组实测数据同时校正上下半区。
 */
static float ArmKinematics_MapDesiredYToCommandY(float desired_y_mm)
{
#if ARM_KIN_Y_AXIS_CALIBRATION_ENABLED
    float sign = 1.0F;
    float desired_abs_mm = desired_y_mm;
    uint8_t index;

    if (desired_abs_mm < 0.0F)
    {
        sign = -1.0F;
        desired_abs_mm = -desired_abs_mm;
    }

    if (desired_abs_mm <= s_y_axis_measured_abs_mm[0])
    {
        return desired_y_mm;
    }

    for (index = 1U;
         index < ARM_KIN_Y_AXIS_CALIBRATION_POINT_COUNT;
         ++index)
    {
        float measured_start =
            s_y_axis_measured_abs_mm[index - 1U];
        float measured_end =
            s_y_axis_measured_abs_mm[index];
        float measured_span = measured_end - measured_start;

        if (measured_span <= ARM_KIN_EPSILON)
        {
            continue;
        }

        if (desired_abs_mm <= measured_end)
        {
            float ratio =
                (desired_abs_mm - measured_start) / measured_span;
            float command_abs_mm =
                s_y_axis_command_abs_mm[index - 1U] +
                ((s_y_axis_command_abs_mm[index] -
                  s_y_axis_command_abs_mm[index - 1U]) * ratio);

            return sign * command_abs_mm;
        }
    }

    return
        sign *
        s_y_axis_command_abs_mm[
            ARM_KIN_Y_AXIS_CALIBRATION_POINT_COUNT - 1U];
#else
    return desired_y_mm;
#endif
}

#if ARM_KIN_XY_RESIDUAL_ENABLED
static uint8_t ArmKinematics_FindGridSegment(float value,
                                             const float *knots,
                                             uint8_t count,
                                             float *ratio)
{
    uint8_t index;

    if (value <= knots[0])
    {
        *ratio = 0.0F;
        return 0U;
    }

    if (value >= knots[count - 1U])
    {
        *ratio = 1.0F;
        return (uint8_t)(count - 2U);
    }

    for (index = 1U; index < count; ++index)
    {
        if (value <= knots[index])
        {
            float span = knots[index] - knots[index - 1U];

            *ratio = (span > ARM_KIN_EPSILON)
                         ? (value - knots[index - 1U]) / span
                         : 0.0F;
            return (uint8_t)(index - 1U);
        }
    }

    *ratio = 1.0F;
    return (uint8_t)(count - 2U);
}

static void ArmKinematics_InterpolateXYForwardError(float command_x_mm,
                                                     float command_y_mm,
                                                     float *error_x_mm,
                                                     float *error_y_mm)
{
    uint8_t x0;
    uint8_t y0;
    uint8_t x1;
    uint8_t y1;
    float tx;
    float ty;
    float ex00;
    float ex10;
    float ex01;
    float ex11;
    float ey00;
    float ey10;
    float ey01;
    float ey11;
    float ex0;
    float ex1;
    float ey0;
    float ey1;

    x0 = ArmKinematics_FindGridSegment(command_x_mm,
                                       s_xy_grid_command_x_mm,
                                       ARM_KIN_XY_GRID_X_COUNT,
                                       &tx);
    y0 = ArmKinematics_FindGridSegment(command_y_mm,
                                       s_xy_grid_command_y_mm,
                                       ARM_KIN_XY_GRID_Y_COUNT,
                                       &ty);
    x1 = (uint8_t)(x0 + 1U);
    y1 = (uint8_t)(y0 + 1U);

    ex00 = s_xy_grid_measured_x_mm[y0][x0] -
           s_xy_grid_command_x_mm[x0];
    ex10 = s_xy_grid_measured_x_mm[y0][x1] -
           s_xy_grid_command_x_mm[x1];
    ex01 = s_xy_grid_measured_x_mm[y1][x0] -
           s_xy_grid_command_x_mm[x0];
    ex11 = s_xy_grid_measured_x_mm[y1][x1] -
           s_xy_grid_command_x_mm[x1];

    ey00 = s_xy_grid_measured_y_mm[y0][x0] -
           s_xy_grid_command_y_mm[y0];
    ey10 = s_xy_grid_measured_y_mm[y0][x1] -
           s_xy_grid_command_y_mm[y0];
    ey01 = s_xy_grid_measured_y_mm[y1][x0] -
           s_xy_grid_command_y_mm[y1];
    ey11 = s_xy_grid_measured_y_mm[y1][x1] -
           s_xy_grid_command_y_mm[y1];

    ex0 = ex00 + ((ex10 - ex00) * tx);
    ex1 = ex01 + ((ex11 - ex01) * tx);
    ey0 = ey00 + ((ey10 - ey00) * tx);
    ey1 = ey01 + ((ey11 - ey01) * tx);

    *error_x_mm = ex0 + ((ex1 - ex0) * ty);
    *error_y_mm = ey0 + ((ey1 - ey0) * ty);
}
#endif

/*
 * 已知实测网格描述的是正向关系 measured=command+error(command)。
 * 这里用固定点迭代求逆，不把“目标-实测”简单加一次，减少X/Y耦合
 * 以及分段斜率造成的二次残差。
 */
static void ArmKinematics_ApplyXYResidualScaled(float *board_x_mm,
                                                float *board_y_mm,
                                                float scale)
{
#if ARM_KIN_XY_RESIDUAL_ENABLED
    float desired_x_mm;
    float desired_y_mm;
    float command_x_mm;
    float command_y_mm;
    uint8_t iteration;

    if ((board_x_mm == NULL) || (board_y_mm == NULL))
    {
        return;
    }

    desired_x_mm = *board_x_mm;
    desired_y_mm = *board_y_mm;
    command_x_mm = desired_x_mm;
    command_y_mm = desired_y_mm;

    for (iteration = 0U;
         iteration < ARM_KIN_XY_GRID_INVERSE_ITERATIONS;
         ++iteration)
    {
        float error_x_mm;
        float error_y_mm;

        ArmKinematics_InterpolateXYForwardError(command_x_mm,
                                                 command_y_mm,
                                                 &error_x_mm,
                                                 &error_y_mm);
        command_x_mm = desired_x_mm - error_x_mm;
        command_y_mm = desired_y_mm - error_y_mm;
    }

    *board_x_mm =
        desired_x_mm + ((command_x_mm - desired_x_mm) * scale);
    *board_y_mm =
        desired_y_mm + ((command_y_mm - desired_y_mm) * scale);
#else
    (void)board_x_mm;
    (void)board_y_mm;
    (void)scale;
#endif
}

static void ArmKinematics_ApplyXYResidual(float *board_x_mm,
                                          float *board_y_mm)
{
    ArmKinematics_ApplyXYResidualScaled(board_x_mm,
                                        board_y_mm,
                                        1.0F);
}

#if ARM_KIN_COORD_CORRECTION_ENABLED
static float ArmKinematics_Lerp(float a, float b, float t)
{
    return a + ((b - a) * t);
}

static float ArmKinematics_Clamp01(float value)
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

static float ArmKinematics_GetSegmentRatio(float value,
                                           float start,
                                           float end)
{
    if ((end - start) < ARM_KIN_EPSILON)
    {
        return 0.0F;
    }

    return ArmKinematics_Clamp01((value - start) / (end - start));
}

static void ArmKinematics_BilinearCorrection(float x_ratio,
                                             float y_ratio,
                                             float dx00,
                                             float dy00,
                                             float dx10,
                                             float dy10,
                                             float dx01,
                                             float dy01,
                                             float dx11,
                                             float dy11,
                                             float *dx,
                                             float *dy)
{
    float dx0 = ArmKinematics_Lerp(dx00, dx10, x_ratio);
    float dx1 = ArmKinematics_Lerp(dx01, dx11, x_ratio);
    float dy0 = ArmKinematics_Lerp(dy00, dy10, x_ratio);
    float dy1 = ArmKinematics_Lerp(dy01, dy11, x_ratio);

    *dx = ArmKinematics_Lerp(dx0, dx1, y_ratio);
    *dy = ArmKinematics_Lerp(dy0, dy1, y_ratio);
}

static void ArmKinematics_ApplyCoordinateCorrectionScaled(
    float *board_x_mm,
    float *board_y_mm,
    float correction_scale)
{
    float x = *board_x_mm;
    float y = *board_y_mm;
    float dx;
    float dy;

    if (x <= ARM_KIN_COORD_CORRECTION_X1_MM)
    {
        float x_ratio =
            ArmKinematics_GetSegmentRatio(
                x,
                ARM_KIN_COORD_CORRECTION_X0_MM,
                ARM_KIN_COORD_CORRECTION_X1_MM);

        if (y <= ARM_KIN_COORD_CORRECTION_Y1_MM)
        {
            float y_ratio =
                ArmKinematics_GetSegmentRatio(
                    y,
                    ARM_KIN_COORD_CORRECTION_Y0_MM,
                    ARM_KIN_COORD_CORRECTION_Y1_MM);

            ArmKinematics_BilinearCorrection(
                x_ratio,
                y_ratio,
                ARM_KIN_COORD_CORRECTION_DX00_MM,
                ARM_KIN_COORD_CORRECTION_DY00_MM,
                ARM_KIN_COORD_CORRECTION_DX10_MM,
                ARM_KIN_COORD_CORRECTION_DY10_MM,
                ARM_KIN_COORD_CORRECTION_DX01_MM,
                ARM_KIN_COORD_CORRECTION_DY01_MM,
                ARM_KIN_COORD_CORRECTION_DX11_MM,
                ARM_KIN_COORD_CORRECTION_DY11_MM,
                &dx,
                &dy);
        }
        else
        {
            float y_ratio =
                ArmKinematics_GetSegmentRatio(
                    y,
                    ARM_KIN_COORD_CORRECTION_Y1_MM,
                    ARM_KIN_COORD_CORRECTION_Y2_MM);

            ArmKinematics_BilinearCorrection(
                x_ratio,
                y_ratio,
                ARM_KIN_COORD_CORRECTION_DX01_MM,
                ARM_KIN_COORD_CORRECTION_DY01_MM,
                ARM_KIN_COORD_CORRECTION_DX11_MM,
                ARM_KIN_COORD_CORRECTION_DY11_MM,
                ARM_KIN_COORD_CORRECTION_DX02_MM,
                ARM_KIN_COORD_CORRECTION_DY02_MM,
                ARM_KIN_COORD_CORRECTION_DX12_MM,
                ARM_KIN_COORD_CORRECTION_DY12_MM,
                &dx,
                &dy);
        }
    }
    else
    {
        float x_ratio =
            ArmKinematics_GetSegmentRatio(
                x,
                ARM_KIN_COORD_CORRECTION_X1_MM,
                ARM_KIN_COORD_CORRECTION_X2_MM);

        if (y <= ARM_KIN_COORD_CORRECTION_Y1_MM)
        {
            float y_ratio =
                ArmKinematics_GetSegmentRatio(
                    y,
                    ARM_KIN_COORD_CORRECTION_Y0_MM,
                    ARM_KIN_COORD_CORRECTION_Y1_MM);

            ArmKinematics_BilinearCorrection(
                x_ratio,
                y_ratio,
                ARM_KIN_COORD_CORRECTION_DX10_MM,
                ARM_KIN_COORD_CORRECTION_DY10_MM,
                ARM_KIN_COORD_CORRECTION_DX20_MM,
                ARM_KIN_COORD_CORRECTION_DY20_MM,
                ARM_KIN_COORD_CORRECTION_DX11_MM,
                ARM_KIN_COORD_CORRECTION_DY11_MM,
                ARM_KIN_COORD_CORRECTION_DX21_MM,
                ARM_KIN_COORD_CORRECTION_DY21_MM,
                &dx,
                &dy);
        }
        else
        {
            float y_ratio =
                ArmKinematics_GetSegmentRatio(
                    y,
                    ARM_KIN_COORD_CORRECTION_Y1_MM,
                    ARM_KIN_COORD_CORRECTION_Y2_MM);

            ArmKinematics_BilinearCorrection(
                x_ratio,
                y_ratio,
                ARM_KIN_COORD_CORRECTION_DX11_MM,
                ARM_KIN_COORD_CORRECTION_DY11_MM,
                ARM_KIN_COORD_CORRECTION_DX21_MM,
                ARM_KIN_COORD_CORRECTION_DY21_MM,
                ARM_KIN_COORD_CORRECTION_DX12_MM,
                ARM_KIN_COORD_CORRECTION_DY12_MM,
                ARM_KIN_COORD_CORRECTION_DX22_MM,
                ARM_KIN_COORD_CORRECTION_DY22_MM,
                &dx,
                &dy);
        }
    }

    *board_x_mm = x + (dx * correction_scale);
    *board_y_mm = y + (dy * correction_scale);
}

static void ArmKinematics_ApplyCoordinateCorrection(float *board_x_mm,
                                                    float *board_y_mm)
{
    ArmKinematics_ApplyCoordinateCorrectionScaled(
        board_x_mm,
        board_y_mm,
        ARM_KIN_COORD_CORRECTION_SCALE);
}
#else
static void ArmKinematics_ApplyCoordinateCorrectionScaled(
    float *board_x_mm,
    float *board_y_mm,
    float correction_scale)
{
    (void)board_x_mm;
    (void)board_y_mm;
    (void)correction_scale;
}

static void ArmKinematics_ApplyCoordinateCorrection(float *board_x_mm,
                                                    float *board_y_mm)
{
    (void)board_x_mm;
    (void)board_y_mm;
}
#endif

/*
 * 应用右半区最新实测残差。command_x_mm必须使用补偿前的用户/视觉
 * 目标X，避免补偿层之间互相改变介入比例。
 */
static void ArmKinematics_ApplyRightResidualCorrection(
    float command_x_mm,
    float *board_x_mm,
    float *board_y_mm)
{
#if ARM_KIN_RIGHT_RESIDUAL_COMP_ENABLED
    float ratio;

    if ((board_x_mm == NULL) || (board_y_mm == NULL) ||
        (command_x_mm <= ARM_KIN_RIGHT_RESIDUAL_START_X_MM))
    {
        return;
    }

    if (command_x_mm >= ARM_KIN_RIGHT_RESIDUAL_FULL_X_MM)
    {
        ratio = 1.0F;
    }
    else
    {
        ratio =
            (command_x_mm - ARM_KIN_RIGHT_RESIDUAL_START_X_MM) /
            (ARM_KIN_RIGHT_RESIDUAL_FULL_X_MM -
             ARM_KIN_RIGHT_RESIDUAL_START_X_MM);
    }

    *board_x_mm += ARM_KIN_RIGHT_RESIDUAL_DX_MM * ratio;
    *board_y_mm += ARM_KIN_RIGHT_RESIDUAL_DY_MM * ratio;
#else
    (void)command_x_mm;
    (void)board_x_mm;
    (void)board_y_mm;
#endif
}

/*
 * 返回当前题面坐标使用的总X偏移。固定偏移修正整体安装误差，P3平滑
 * 补偿只修正右边中部的伸出不足，并在右上/右下角衰减为零。
 */
static float ArmKinematics_GetTargetXOffset(float board_x_mm,
                                             float board_y_mm)
{
    float offset_mm = ARM_KIN_TARGET_X_OFFSET_MM;

#if ARM_KIN_P3_X_COMP_ENABLED
    float x_ratio;
    float y_ratio;
    float absolute_y_mm = fabsf(board_y_mm);

    if (board_x_mm <= ARM_KIN_P3_X_COMP_START_X_MM)
    {
        x_ratio = 0.0F;
    }
    else if (board_x_mm >= ARM_KIN_P3_X_COMP_FULL_X_MM)
    {
        x_ratio = 1.0F;
    }
    else
    {
        x_ratio =
            (board_x_mm - ARM_KIN_P3_X_COMP_START_X_MM) /
            (ARM_KIN_P3_X_COMP_FULL_X_MM -
             ARM_KIN_P3_X_COMP_START_X_MM);
    }

    if (absolute_y_mm >= ARM_KIN_P3_X_COMP_Y_FADE_MM)
    {
        y_ratio = 0.0F;
    }
    else
    {
        y_ratio =
            1.0F -
            (absolute_y_mm / ARM_KIN_P3_X_COMP_Y_FADE_MM);
    }

    offset_mm +=
        ARM_KIN_P3_X_COMP_MAX_MM * x_ratio * y_ratio;
#endif

#if ARM_KIN_X_MID_COMP_ENABLED
    {
        float mid_x_ratio;
        float mid_y_ratio;
        float distance_from_center_mm =
            fabsf(board_x_mm - ARM_KIN_X_MID_COMP_CENTER_MM);
        float absolute_y_mm = fabsf(board_y_mm);

        if (distance_from_center_mm >= ARM_KIN_X_MID_COMP_HALF_WIDTH_MM)
        {
            mid_x_ratio = 0.0F;
        }
        else
        {
            mid_x_ratio =
                1.0F -
                (distance_from_center_mm /
                 ARM_KIN_X_MID_COMP_HALF_WIDTH_MM);
        }

        if (absolute_y_mm >= ARM_KIN_X_MID_COMP_Y_FADE_MM)
        {
            mid_y_ratio = 0.0F;
        }
        else
        {
            mid_y_ratio =
                1.0F -
                (absolute_y_mm / ARM_KIN_X_MID_COMP_Y_FADE_MM);
        }

        offset_mm +=
            ARM_KIN_X_MID_COMP_MAX_MM * mid_x_ratio * mid_y_ratio;
    }
#endif

    return offset_mm;
}

bool ArmKinematics_GetBaseYawDeg(float board_x_mm,
                                 float board_y_mm,
                                 float *base_yaw_deg)
{
    float command_x_mm;
    float dx;
    float dy;

    if (base_yaw_deg == NULL)
    {
        return false;
    }

    ArmKinematics_ApplyXYResidual(&board_x_mm, &board_y_mm);
    board_x_mm = ArmKinematics_MapDesiredXToCommandX(board_x_mm);
    board_y_mm = ArmKinematics_MapDesiredYToCommandY(board_y_mm);
    command_x_mm = board_x_mm;

    ArmKinematics_ApplyCoordinateCorrection(&board_x_mm, &board_y_mm);
    ArmKinematics_ApplyRightResidualCorrection(command_x_mm,
                                               &board_x_mm,
                                               &board_y_mm);

    dx =
        board_x_mm +
        ArmKinematics_GetTargetXOffset(board_x_mm, board_y_mm) -
        ARM_KIN_BASE_X_MM;
    dy = board_y_mm - ARM_KIN_BASE_Y_MM;

    if (((dx > -ARM_KIN_EPSILON) && (dx < ARM_KIN_EPSILON)) &&
        ((dy > -ARM_KIN_EPSILON) && (dy < ARM_KIN_EPSILON)))
    {
        return false;
    }

    /*
     * 机械臂位于A4左侧，正前方定义为题面+X，向题面+Y旋转为正。
     * 从纸面上方向下观察时，+X转向+Y是顺时针，与视觉协议一致。
     */
    *base_yaw_deg = atan2f(dy, dx) * ARM_KIN_RAD_TO_DEG;
    return true;
}

static bool ArmKinematics_BuildSolution(float board_x_mm,
                                        float board_y_mm,
                                        float base_yaw_deg,
                                        float shoulder_rad,
                                        float elbow_rad,
                                        float wrist_rad,
                                        float magnet_relative_deg,
                                        ArmKinematicsSolution *solution)
{
    int16_t angles[ARM_SERVO_COUNT];
    uint8_t index;

    /*
     * 三个俯仰关节必须分别把数学关节角映射成实际舵机角。
     * 工具绝对俯仰已经由 q_wrist = tool_pitch-q_shoulder-q_elbow
     * 保证，不能再强制“舵机显示角之和恒定”。J3存在非1:1比例时，
     * 强制显示角之和恒定反而会破坏物理关节角之和，导致远近位置倾斜。
     */
    angles[ARM_KIN_SERVO_MAGNET] =
        ArmKinematics_MapJoint(magnet_relative_deg,
                               ARM_KIN_MAGNET_ZERO_ANGLE_X10,
                               ARM_KIN_MAGNET_DIRECTION);
    angles[ARM_KIN_SERVO_WRIST] =
        ArmKinematics_MapJoint(wrist_rad * ARM_KIN_RAD_TO_DEG,
                               ARM_KIN_WRIST_ZERO_ANGLE_X10,
                               ARM_KIN_WRIST_DIRECTION);
    angles[ARM_KIN_SERVO_ELBOW] =
        ArmKinematics_MapJointScaled(elbow_rad * ARM_KIN_RAD_TO_DEG,
                                     ARM_KIN_ELBOW_ZERO_ANGLE_X10,
                                     ARM_KIN_ELBOW_DIRECTION,
                                     ARM_KIN_ELBOW_GAIN);
    angles[ARM_KIN_SERVO_SHOULDER] =
        ArmKinematics_MapJoint(shoulder_rad * ARM_KIN_RAD_TO_DEG,
                               ARM_KIN_SHOULDER_ZERO_ANGLE_X10,
                               ARM_KIN_SHOULDER_DIRECTION);
    angles[ARM_KIN_SERVO_BASE] =
        ArmKinematics_MapJoint(base_yaw_deg,
                               ARM_KIN_BASE_FORWARD_ANGLE_X10,
                               ARM_KIN_BASE_DIRECTION);

    ArmKinematics_ApplyBoardCorrection(board_x_mm,
                                       board_y_mm,
                                       angles);

    if (!ArmServo_ValidateAngles(angles))
    {
        return false;
    }

    for (index = 0U; index < ARM_SERVO_COUNT; ++index)
    {
        solution->servo_angle_x10[index] = angles[index];
    }

    solution->base_yaw_deg = base_yaw_deg;
    solution->shoulder_deg = shoulder_rad * ARM_KIN_RAD_TO_DEG;
    solution->elbow_deg = elbow_rad * ARM_KIN_RAD_TO_DEG;
    solution->wrist_deg = wrist_rad * ARM_KIN_RAD_TO_DEG;
    return true;
}

bool ArmKinematics_SolveBoardPose(float board_x_mm,
                                  float board_y_mm,
                                  float board_z_mm,
                                  float magnet_relative_deg,
                                  ArmKinematicsSolution *solution)
{
    float correction_scale;
    float requested_x_mm;
    float requested_y_mm;

    if (solution == NULL)
    {
        return false;
    }

    requested_x_mm = board_x_mm;
    requested_y_mm = board_y_mm;
    board_z_mm -= ArmKinematics_GetXZCompensation(requested_x_mm);
    board_z_mm -=
        ArmKinematics_GetLocalZCompensation(requested_x_mm,
                                            requested_y_mm);
    board_z_mm -= ArmKinematics_GetNegYZCompensation(requested_y_mm);

    for (correction_scale = ARM_KIN_COORD_CORRECTION_SCALE;
         correction_scale >= -ARM_KIN_EPSILON;
         correction_scale -= ARM_KIN_COORD_CORRECTION_RETRY_STEP)
    {
        float solved_x_mm = requested_x_mm;
        float solved_y_mm = requested_y_mm;
        float command_x_mm;
        float dx;
        float dy;
        float base_radius;
        float planar_x;
        float planar_z;
        float tool_pitch_rad;
        float wrist_x;
        float wrist_z;
        float cosine_elbow;
        float sine_elbow_abs;
        float base_yaw_deg;
        int8_t branch;

        if (correction_scale < 0.0F)
        {
            correction_scale = 0.0F;
        }

        ArmKinematics_ApplyXYResidualScaled(&solved_x_mm,
                                            &solved_y_mm,
                                            correction_scale);
        solved_x_mm =
            ArmKinematics_MapDesiredXToCommandX(solved_x_mm);
        solved_y_mm =
            ArmKinematics_MapDesiredYToCommandY(solved_y_mm);
        command_x_mm = solved_x_mm;

        ArmKinematics_ApplyCoordinateCorrectionScaled(
            &solved_x_mm,
            &solved_y_mm,
            correction_scale);
        ArmKinematics_ApplyRightResidualCorrection(command_x_mm,
                                                   &solved_x_mm,
                                                   &solved_y_mm);

        dx =
            solved_x_mm +
            ArmKinematics_GetTargetXOffset(solved_x_mm, solved_y_mm) -
            ARM_KIN_BASE_X_MM;
        dy = solved_y_mm - ARM_KIN_BASE_Y_MM;

        if (((dx > -ARM_KIN_EPSILON) && (dx < ARM_KIN_EPSILON)) &&
            ((dy > -ARM_KIN_EPSILON) && (dy < ARM_KIN_EPSILON)))
        {
            continue;
        }

        base_yaw_deg = atan2f(dy, dx) * ARM_KIN_RAD_TO_DEG;
        base_radius = sqrtf((dx * dx) + (dy * dy));
        planar_x = base_radius - ARM_KIN_BASE_TO_SHOULDER_MM;
        /*
         * board_z_mm以A4纸面为零；肩轴高度以底座安装面为零。
         * A4纸面下方的垫高必须先补偿，否则所有末端目标都会偏低。
         */
        planar_z =
            board_z_mm + ARM_KIN_BOARD_PLANE_HEIGHT_MM -
            ARM_KIN_SHOULDER_HEIGHT_MM;

        if (planar_x <= ARM_KIN_EPSILON)
        {
            continue;
        }

        tool_pitch_rad =
            (ARM_KIN_TOOL_PITCH_DEG +
             ArmKinematics_GetFarPointPitchCompensation(base_radius)) *
            ARM_KIN_DEG_TO_RAD;
        wrist_x =
            planar_x -
            (ARM_KIN_LINK_WRIST_TOOL_MM * cosf(tool_pitch_rad));
        wrist_z =
            planar_z -
            (ARM_KIN_LINK_WRIST_TOOL_MM * sinf(tool_pitch_rad));

        cosine_elbow =
            ((wrist_x * wrist_x) + (wrist_z * wrist_z) -
             (ARM_KIN_LINK_SHOULDER_ELBOW_MM *
              ARM_KIN_LINK_SHOULDER_ELBOW_MM) -
             (ARM_KIN_LINK_ELBOW_WRIST_MM *
              ARM_KIN_LINK_ELBOW_WRIST_MM)) /
            (2.0F * ARM_KIN_LINK_SHOULDER_ELBOW_MM *
             ARM_KIN_LINK_ELBOW_WRIST_MM);

        if ((cosine_elbow < (-1.0F - ARM_KIN_EPSILON)) ||
            (cosine_elbow > (1.0F + ARM_KIN_EPSILON)))
        {
            continue;
        }

        if (cosine_elbow > 1.0F)
        {
            cosine_elbow = 1.0F;
        }
        else if (cosine_elbow < -1.0F)
        {
            cosine_elbow = -1.0F;
        }

        sine_elbow_abs =
            sqrtf(1.0F - (cosine_elbow * cosine_elbow));

        /*
         * 实机J3从0°增大时，前臂向A4方向弯曲，对应数学负肘分支。
         * 因此先尝试负肘，再尝试正肘。只有转换成实际五路舵机角后全部
         * 落入机械限位的分支才会返回，禁止用裁剪角度伪装成可达姿态。
         */
        for (branch = -1; branch <= 1; branch += 2)
        {
            float elbow_rad =
                atan2f((float)branch * sine_elbow_abs, cosine_elbow);
            float shoulder_rad =
                atan2f(wrist_z, wrist_x) -
                atan2f(ARM_KIN_LINK_ELBOW_WRIST_MM * sinf(elbow_rad),
                       ARM_KIN_LINK_SHOULDER_ELBOW_MM +
                       (ARM_KIN_LINK_ELBOW_WRIST_MM * cosf(elbow_rad)));
            float wrist_rad =
                tool_pitch_rad - shoulder_rad - elbow_rad;

            if (ArmKinematics_BuildSolution(solved_x_mm,
                                            solved_y_mm,
                                            base_yaw_deg,
                                            shoulder_rad,
                                            elbow_rad,
                                            wrist_rad,
                                            magnet_relative_deg,
                                            solution))
            {
                return true;
            }
        }
    }

    return false;
}
