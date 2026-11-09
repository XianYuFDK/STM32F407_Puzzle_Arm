#include "arm_servo.h"

#include "bsp_servo_pwm.h"

typedef struct
{
    int16_t min_angle_x10;
    int16_t max_angle_x10;
    int16_t home_angle_x10;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    bool reversed;
} ArmServoCalibration;

static const ArmServoCalibration s_calibration[ARM_SERVO_COUNT] =
{
    {
        ARM_SERVO1_MIN_ANGLE_X10,
        ARM_SERVO1_MAX_ANGLE_X10,
        ARM_SERVO1_HOME_ANGLE_X10,
        ARM_SERVO1_MIN_PULSE_US,
        ARM_SERVO1_MAX_PULSE_US,
        (ARM_SERVO1_REVERSED != 0U)
    },
    {
        ARM_SERVO2_MIN_ANGLE_X10,
        ARM_SERVO2_MAX_ANGLE_X10,
        ARM_SERVO2_HOME_ANGLE_X10,
        ARM_SERVO2_MIN_PULSE_US,
        ARM_SERVO2_MAX_PULSE_US,
        (ARM_SERVO2_REVERSED != 0U)
    },
    {
        ARM_SERVO3_MIN_ANGLE_X10,
        ARM_SERVO3_MAX_ANGLE_X10,
        ARM_SERVO3_HOME_ANGLE_X10,
        ARM_SERVO3_MIN_PULSE_US,
        ARM_SERVO3_MAX_PULSE_US,
        (ARM_SERVO3_REVERSED != 0U)
    },
    {
        ARM_SERVO4_MIN_ANGLE_X10,
        ARM_SERVO4_MAX_ANGLE_X10,
        ARM_SERVO4_HOME_ANGLE_X10,
        ARM_SERVO4_MIN_PULSE_US,
        ARM_SERVO4_MAX_PULSE_US,
        (ARM_SERVO4_REVERSED != 0U)
    },
    {
        ARM_SERVO5_MIN_ANGLE_X10,
        ARM_SERVO5_MAX_ANGLE_X10,
        ARM_SERVO5_HOME_ANGLE_X10,
        ARM_SERVO5_MIN_PULSE_US,
        ARM_SERVO5_MAX_PULSE_US,
        (ARM_SERVO5_REVERSED != 0U)
    }
};

static int16_t s_current_angle_x10[ARM_SERVO_COUNT];

static bool ArmServo_AngleToPulse(uint8_t servo_index,
                                  int16_t angle_x10,
                                  uint16_t *pulse_us)
{
    const ArmServoCalibration *calibration;
    int32_t angle_offset;
    int32_t angle_range;
    int32_t pulse_range;
    int32_t mapped_pulse;

    if ((servo_index >= ARM_SERVO_COUNT) || (pulse_us == NULL))
    {
        return false;
    }

    calibration = &s_calibration[servo_index];

    if ((angle_x10 < calibration->min_angle_x10) ||
        (angle_x10 > calibration->max_angle_x10) ||
        (calibration->max_angle_x10 <= calibration->min_angle_x10) ||
        (calibration->max_pulse_us <= calibration->min_pulse_us))
    {
        return false;
    }

    angle_range = (int32_t)calibration->max_angle_x10 -
                  (int32_t)calibration->min_angle_x10;

    if (calibration->reversed)
    {
        angle_offset = (int32_t)calibration->max_angle_x10 -
                       (int32_t)angle_x10;
    }
    else
    {
        angle_offset = (int32_t)angle_x10 -
                       (int32_t)calibration->min_angle_x10;
    }

    pulse_range = (int32_t)calibration->max_pulse_us -
                  (int32_t)calibration->min_pulse_us;
    mapped_pulse = (int32_t)calibration->min_pulse_us +
                   ((angle_offset * pulse_range) / angle_range);

    *pulse_us = (uint16_t)mapped_pulse;
    return true;
}

bool ArmServo_Init(void)
{
    uint8_t index;
    uint16_t pulse_us;

    if (!BspServoPwm_Init())
    {
        return false;
    }

#if ARM_ZERO_CALIBRATION_MODE
    /*
     * 调零时只输出一路固定中位脉冲。未选中的通道比较值为 0，
     * 即整周期保持低电平，避免其他已接电舵机意外运动。
     */
    for (index = 0U; index < ARM_SERVO_COUNT; ++index)
    {
        pulse_us = ((uint8_t)(index + 1U) ==
                    (uint8_t)ARM_ZERO_CALIBRATION_SERVO)
                       ? ARM_ZERO_CALIBRATION_PULSE_US
                       : 0U;

        if (!BspServoPwm_SetPulseUs(index, pulse_us))
        {
            return false;
        }

        s_current_angle_x10[index] = s_calibration[index].home_angle_x10;
    }

    return BspServoPwm_Start();
#else
    /*
     * 先写入所有舵机的 HOME 比较值，再统一启动 PWM，避免启动时先输出
     * 默认值、随后又跳到 HOME 位置。
     */
    for (index = 0U; index < ARM_SERVO_COUNT; ++index)
    {
        if (!ArmServo_AngleToPulse(index,
                                   s_calibration[index].home_angle_x10,
                                   &pulse_us))
        {
            return false;
        }

        if (!BspServoPwm_SetPulseUs(index, pulse_us))
        {
            return false;
        }

        s_current_angle_x10[index] = s_calibration[index].home_angle_x10;
    }

    return BspServoPwm_Start();
#endif
}

bool ArmServo_ValidateAngles(const int16_t angle_x10[ARM_SERVO_COUNT])
{
    uint8_t index;

    if (angle_x10 == NULL)
    {
        return false;
    }

    for (index = 0U; index < ARM_SERVO_COUNT; ++index)
    {
        if ((angle_x10[index] < s_calibration[index].min_angle_x10) ||
            (angle_x10[index] > s_calibration[index].max_angle_x10))
        {
            return false;
        }
    }

    return true;
}

bool ArmServo_SetAngle(uint8_t servo_index, int16_t angle_x10)
{
    uint16_t pulse_us;

    if (!ArmServo_AngleToPulse(servo_index, angle_x10, &pulse_us))
    {
        return false;
    }

    if (!BspServoPwm_SetPulseUs(servo_index, pulse_us))
    {
        return false;
    }

    s_current_angle_x10[servo_index] = angle_x10;
    return true;
}

bool ArmServo_SetAngles(const int16_t angle_x10[ARM_SERVO_COUNT])
{
    uint8_t index;

    /* 全部检查通过后再输出，防止部分关节已经移动、后续关节却超限。 */
    if (!ArmServo_ValidateAngles(angle_x10))
    {
        return false;
    }

    for (index = 0U; index < ARM_SERVO_COUNT; ++index)
    {
        if (!ArmServo_SetAngle(index, angle_x10[index]))
        {
            return false;
        }
    }

    return true;
}

void ArmServo_GetAngles(int16_t angle_x10[ARM_SERVO_COUNT])
{
    uint8_t index;

    if (angle_x10 == NULL)
    {
        return;
    }

    for (index = 0U; index < ARM_SERVO_COUNT; ++index)
    {
        angle_x10[index] = s_current_angle_x10[index];
    }
}

void ArmServo_GetHomeAngles(int16_t angle_x10[ARM_SERVO_COUNT])
{
    uint8_t index;

    if (angle_x10 == NULL)
    {
        return;
    }

    for (index = 0U; index < ARM_SERVO_COUNT; ++index)
    {
        angle_x10[index] = s_calibration[index].home_angle_x10;
    }
}
