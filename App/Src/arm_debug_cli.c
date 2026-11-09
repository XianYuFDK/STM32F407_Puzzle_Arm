#include "arm_debug_cli.h"

#include <stdio.h>
#include <string.h>

#include "arm_calibration.h"
#include "arm_comm.h"
#include "arm_motion.h"
#include "arm_kinematics.h"
#include "arm_puzzle.h"
#include "arm_puzzle_protocol.h"
#include "arm_protocol.h"
#include "arm_servo.h"
#include "bsp_debug_uart_dma.h"
#include "bsp_magnet.h"
#include "bsp_safety_io.h"
#include "bsp_uart_dma.h"

#define ARM_DEBUG_CLI_MAX_TOKENS       8U
#define ARM_DEBUG_CLI_RESPONSE_SIZE    320U

static char s_line[ARM_DEBUG_CLI_LINE_SIZE];
static uint16_t s_line_length;
static uint8_t s_sequence;

static void ArmDebugCli_Send(const char *text)
{
    size_t length;

    if (text == NULL)
    {
        return;
    }

    length = strlen(text);
    if (length > 0xFFFFU)
    {
        length = 0xFFFFU;
    }

    (void)BspDebugUartDma_Transmit((const uint8_t *)text,
                                   (uint16_t)length,
                                   100U);
}

static const char *ArmDebugCli_ResultName(ArmResult result)
{
    switch (result)
    {
        case ARM_RESULT_OK:
            return "OK";
        case ARM_RESULT_BAD_LENGTH:
            return "BAD_LENGTH";
        case ARM_RESULT_BAD_VERSION:
            return "BAD_VERSION";
        case ARM_RESULT_BAD_CRC:
            return "BAD_CRC";
        case ARM_RESULT_BAD_COMMAND:
            return "BAD_COMMAND";
        case ARM_RESULT_ANGLE_LIMIT:
            return "ANGLE_LIMIT";
        case ARM_RESULT_QUEUE_FULL:
            return "QUEUE_FULL";
        case ARM_RESULT_BUSY:
            return "BUSY";
        case ARM_RESULT_HW_ERROR:
            return "HW_ERROR";
        case ARM_RESULT_STOPPED:
            return "STOPPED";
        case ARM_RESULT_ESTOP:
            return "ESTOP";
        case ARM_RESULT_MAGNET_TIMEOUT:
            return "MAGNET_TIMEOUT";
        case ARM_RESULT_COMM_TIMEOUT:
            return "COMM_TIMEOUT";
        case ARM_RESULT_BAD_PARAMETER:
            return "BAD_PARAMETER";
        case ARM_RESULT_IK_UNREACHABLE:
            return "IK_UNREACHABLE";
        case ARM_RESULT_PLAN_TIMEOUT:
            return "PLAN_TIMEOUT";
        default:
            return "UNKNOWN";
    }
}

static void ArmDebugCli_SendResult(const char *command, ArmResult result)
{
    char response[96];

    (void)snprintf(response,
                   sizeof(response),
                   "%s %s %s (%u)\r\n",
                   (result == ARM_RESULT_OK) ? "OK" : "ERR",
                   command,
                   ArmDebugCli_ResultName(result),
                   (unsigned int)result);
    ArmDebugCli_Send(response);
}

static void ArmDebugCli_ToUpper(char *text)
{
    while ((text != NULL) && (*text != '\0'))
    {
        if ((*text >= 'a') && (*text <= 'z'))
        {
            *text = (char)(*text - ('a' - 'A'));
        }
        ++text;
    }
}

static uint8_t ArmDebugCli_Tokenize(char *line, char *tokens[])
{
    uint8_t count = 0U;
    char *cursor = line;

    while ((*cursor != '\0') && (count < ARM_DEBUG_CLI_MAX_TOKENS))
    {
        while ((*cursor == ' ') || (*cursor == '\t'))
        {
            ++cursor;
        }

        if (*cursor == '\0')
        {
            break;
        }

        tokens[count++] = cursor;

        while ((*cursor != '\0') &&
               (*cursor != ' ') &&
               (*cursor != '\t'))
        {
            ++cursor;
        }

        if (*cursor != '\0')
        {
            *cursor = '\0';
            ++cursor;
        }
    }

    return count;
}

static bool ArmDebugCli_ParseU16(const char *text, uint16_t *value)
{
    uint32_t parsed = 0U;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
    {
        return false;
    }

    while (*text != '\0')
    {
        if ((*text < '0') || (*text > '9'))
        {
            return false;
        }

        parsed = (parsed * 10U) + (uint32_t)(*text - '0');
        if (parsed > 0xFFFFU)
        {
            return false;
        }
        ++text;
    }

    *value = (uint16_t)parsed;
    return true;
}

/* 将 "90"、"90.0"、"-10.5" 转换为0.1度单位。 */
static bool ArmDebugCli_ParseAngleX10(const char *text, int16_t *angle_x10)
{
    bool negative = false;
    bool has_digit = false;
    int32_t whole = 0;
    int32_t fraction = 0;
    int32_t value;

    if ((text == NULL) || (angle_x10 == NULL) || (*text == '\0'))
    {
        return false;
    }

    if ((*text == '+') || (*text == '-'))
    {
        negative = (*text == '-');
        ++text;
    }

    while ((*text >= '0') && (*text <= '9'))
    {
        has_digit = true;
        whole = (whole * 10) + (int32_t)(*text - '0');
        if (whole > 3276)
        {
            return false;
        }
        ++text;
    }

    if (*text == '.')
    {
        ++text;
        if ((*text < '0') || (*text > '9'))
        {
            return false;
        }
        fraction = (int32_t)(*text - '0');
        ++text;

        /* 调试协议的角度分辨率为0.1度，只允许一位小数。 */
        if (*text != '\0')
        {
            return false;
        }
    }
    else if (*text != '\0')
    {
        return false;
    }

    if (!has_digit)
    {
        return false;
    }

    value = (whole * 10) + fraction;
    if (negative)
    {
        value = -value;
    }

    if ((value < -32768) || (value > 32767))
    {
        return false;
    }

    *angle_x10 = (int16_t)value;
    return true;
}

static uint8_t ArmDebugCli_NextSequence(void)
{
    ++s_sequence;
    if (s_sequence == 0U)
    {
        ++s_sequence;
    }
    return s_sequence;
}

static ArmResult ArmDebugCli_SubmitMove(
    const int16_t target_angle_x10[ARM_SERVO_COUNT],
    uint16_t duration_ms,
    uint8_t source_command)
{
    ArmMotionCommand command;
    uint8_t joint;

    if (ArmCalibration_IsBusy() ||
        ArmPuzzle_IsBusy() ||
        ArmMotion_IsBusy())
    {
        return ARM_RESULT_BUSY;
    }

    command.sequence = ArmDebugCli_NextSequence();
    command.source_command = source_command;
    command.response_port = ARM_COMM_PORT_DEBUG;
    command.duration_ms = duration_ms;
    command.stop_epoch = 0U;

    for (joint = 0U; joint < ARM_SERVO_COUNT; ++joint)
    {
        command.target_angle_x10[joint] = target_angle_x10[joint];
    }

    return ArmMotion_Submit(&command);
}

static bool ArmDebugCli_HandleServo(char *tokens[], uint8_t token_count)
{
    uint16_t servo_number;
    uint16_t duration_ms = ARM_MOTION_DEFAULT_DURATION_MS;
    int16_t angle_x10;
    int16_t targets[ARM_SERVO_COUNT];
    ArmResult result;
    char response[128];

    if ((token_count != 3U) && (token_count != 4U))
    {
        ArmDebugCli_Send("ERR USAGE: SERVO <1-5> <ANGLE> [TIME_MS]\r\n");
        return true;
    }

    if (!ArmDebugCli_ParseU16(tokens[1], &servo_number) ||
        (servo_number < 1U) ||
        (servo_number > ARM_SERVO_COUNT) ||
        !ArmDebugCli_ParseAngleX10(tokens[2], &angle_x10) ||
        ((token_count == 4U) &&
         !ArmDebugCli_ParseU16(tokens[3], &duration_ms)))
    {
        ArmDebugCli_Send("ERR SERVO BAD_PARAMETER\r\n");
        return true;
    }

    ArmServo_GetAngles(targets);
    targets[servo_number - 1U] = angle_x10;
    result = ArmDebugCli_SubmitMove(targets,
                                    duration_ms,
                                    ARM_CMD_MOVE_JOINTS);
    if (result != ARM_RESULT_OK)
    {
        ArmDebugCli_SendResult("SERVO", result);
        return true;
    }

    (void)snprintf(response,
                   sizeof(response),
                   "OK SERVO %u TARGET=%d.%u TIME=%u\r\n",
                   (unsigned int)servo_number,
                   (int)(angle_x10 / 10),
                   (unsigned int)((angle_x10 < 0)
                                      ? -(angle_x10 % 10)
                                      : (angle_x10 % 10)),
                   (unsigned int)duration_ms);
    ArmDebugCli_Send(response);
    return true;
}

static bool ArmDebugCli_HandleBase(char *tokens[], uint8_t token_count)
{
    char *servo_tokens[4];

    if ((token_count != 2U) && (token_count != 3U))
    {
        ArmDebugCli_Send("ERR USAGE: BASE <ANGLE> [TIME_MS]\r\n");
        return true;
    }

    servo_tokens[0] = "SERVO";
    servo_tokens[1] = "5";
    servo_tokens[2] = tokens[1];
    if (token_count == 3U)
    {
        servo_tokens[3] = tokens[2];
    }

    return ArmDebugCli_HandleServo(servo_tokens,
                                   (uint8_t)(token_count + 1U));
}

static bool ArmDebugCli_HandleMove(char *tokens[], uint8_t token_count)
{
    int16_t targets[ARM_SERVO_COUNT];
    uint16_t duration_ms = ARM_MOTION_DEFAULT_DURATION_MS;
    uint8_t joint;
    ArmResult result;

    if ((token_count != 6U) && (token_count != 7U))
    {
        ArmDebugCli_Send(
            "ERR USAGE: MOVE <J1> <J2> <J3> <J4> <J5> [TIME_MS]\r\n");
        return true;
    }

    for (joint = 0U; joint < ARM_SERVO_COUNT; ++joint)
    {
        if (!ArmDebugCli_ParseAngleX10(tokens[joint + 1U],
                                      &targets[joint]))
        {
            ArmDebugCli_Send("ERR MOVE BAD_ANGLE\r\n");
            return true;
        }
    }

    if ((token_count == 7U) &&
        !ArmDebugCli_ParseU16(tokens[6], &duration_ms))
    {
        ArmDebugCli_Send("ERR MOVE BAD_TIME\r\n");
        return true;
    }

    result = ArmDebugCli_SubmitMove(targets,
                                    duration_ms,
                                    ARM_CMD_MOVE_JOINTS);
    ArmDebugCli_SendResult("MOVE", result);
    return true;
}

static bool ArmDebugCli_HandleHome(char *tokens[], uint8_t token_count)
{
    int16_t targets[ARM_SERVO_COUNT];
    uint16_t duration_ms = ARM_MOTION_DEFAULT_DURATION_MS;
    ArmResult result;

    if ((token_count != 1U) && (token_count != 2U))
    {
        ArmDebugCli_Send("ERR USAGE: HOME [TIME_MS]\r\n");
        return true;
    }

    if ((token_count == 2U) &&
        !ArmDebugCli_ParseU16(tokens[1], &duration_ms))
    {
        ArmDebugCli_Send("ERR HOME BAD_TIME\r\n");
        return true;
    }

    ArmServo_GetHomeAngles(targets);
    result = ArmDebugCli_SubmitMove(targets,
                                    duration_ms,
                                    ARM_CMD_HOME);
    ArmDebugCli_SendResult("HOME", result);
    return true;
}

static bool ArmDebugCli_HandleMagnet(char *tokens[], uint8_t token_count)
{
    bool turn_on;

    if (token_count != 2U)
    {
        ArmDebugCli_Send("ERR USAGE: MAGNET <ON|OFF>\r\n");
        return true;
    }

    if (strcmp(tokens[1], "ON") == 0)
    {
        turn_on = true;
    }
    else if (strcmp(tokens[1], "OFF") == 0)
    {
        turn_on = false;
    }
    else
    {
        ArmDebugCli_Send("ERR MAGNET BAD_PARAMETER\r\n");
        return true;
    }

    BspMagnet_Set(turn_on);
    ArmDebugCli_Send(turn_on ? "OK MAGNET ON\r\n" : "OK MAGNET OFF\r\n");
    return true;
}

static bool ArmDebugCli_HandleStatus(uint8_t token_count)
{
    int16_t angles[ARM_SERVO_COUNT];
    uint32_t vision_rx_events = BspUartDma_GetRxEventCount();
    uint32_t vision_rx_bytes = BspUartDma_GetRxByteCount();
    uint32_t vision_dropped = BspUartDma_GetDroppedChunkCount();
    uint32_t vision_errors = BspUartDma_GetErrorCount();
    uint32_t vision_valid_frames;
    uint32_t vision_invalid_frames;
    uint8_t active_vision_task;
    uint32_t debug_dropped = BspDebugUartDma_GetDroppedChunkCount();
    uint32_t debug_errors = BspDebugUartDma_GetErrorCount();
    ArmPuzzleStatus puzzle_status;
    char response[ARM_DEBUG_CLI_RESPONSE_SIZE];

    if (token_count != 1U)
    {
        ArmDebugCli_Send("ERR USAGE: STATUS\r\n");
        return true;
    }

    ArmServo_GetAngles(angles);
    ArmPuzzle_GetStatus(&puzzle_status);
    ArmComm_GetVisionProtocolStats(&vision_valid_frames,
                                   &vision_invalid_frames,
                                   &active_vision_task);
    (void)snprintf(
        response,
        sizeof(response),
        "STATUS MOVING=%u MAGNET=%u ESTOP=%u "
        "J1=%d.%u J2=%d.%u J3=%d.%u J4=%d.%u J5=%d.%u "
        "V_RX=%lu V_BYTES=%lu V_DROP=%lu V_ERR=%lu "
        "V_TASK=%u V_FRAME=%lu V_BAD=%lu "
        "D_DROP=%lu D_ERR=%lu "
        "PUZZLE=%s PSEQ=%u PIECE=%u PRESULT=%u\r\n",
        ArmMotion_IsMoving() ? 1U : 0U,
        BspMagnet_IsOn() ? 1U : 0U,
        BspSafetyIo_IsEmergencyStopActive() ? 1U : 0U,
        (int)(angles[0] / 10), (unsigned int)(angles[0] % 10),
        (int)(angles[1] / 10), (unsigned int)(angles[1] % 10),
        (int)(angles[2] / 10), (unsigned int)(angles[2] % 10),
        (int)(angles[3] / 10), (unsigned int)(angles[3] % 10),
        (int)(angles[4] / 10), (unsigned int)(angles[4] % 10),
        (unsigned long)vision_rx_events,
        (unsigned long)vision_rx_bytes,
        (unsigned long)vision_dropped,
        (unsigned long)vision_errors,
        (unsigned int)active_vision_task,
        (unsigned long)vision_valid_frames,
        (unsigned long)vision_invalid_frames,
        (unsigned long)debug_dropped,
        (unsigned long)debug_errors,
        ArmPuzzle_StateName(puzzle_status.state),
        (unsigned int)puzzle_status.sequence,
        (unsigned int)puzzle_status.current_piece,
        (unsigned int)puzzle_status.last_result);
    ArmDebugCli_Send(response);
    return true;
}

static bool ArmDebugCli_HandleTask1(uint8_t token_count)
{
    if (token_count != 1U)
    {
        ArmDebugCli_Send("ERR USAGE: TASK1\r\n");
        return true;
    }

    if (ArmCalibration_IsBusy() ||
        ArmPuzzle_IsBusy() ||
        ArmMotion_IsBusy())
    {
        ArmDebugCli_Send("ERR TASK1 BUSY\r\n");
        return true;
    }

    if (!ArmComm_StartVisionTask1())
    {
        ArmDebugCli_Send("ERR TASK1 USART1_TX_FAILED\r\n");
        return true;
    }

    ArmDebugCli_Send("OK TASK1 TX=3A WAIT_VISION_FRAME\r\n");
    return true;
}

static bool ArmDebugCli_HandleTask2(uint8_t token_count)
{
    if (token_count != 1U)
    {
        ArmDebugCli_Send("ERR USAGE: TASK2\r\n");
        return true;
    }

    if (ArmCalibration_IsBusy() ||
        ArmPuzzle_IsBusy() ||
        ArmMotion_IsBusy())
    {
        ArmDebugCli_Send("ERR TASK2 BUSY\r\n");
        return true;
    }

    if (!ArmComm_StartVisionTask2())
    {
        ArmDebugCli_Send("ERR TASK2 USART1_TX_FAILED\r\n");
        return true;
    }

    ArmDebugCli_Send("OK TASK2 TX=3B WAIT_VISION_FRAME\r\n");
    return true;
}

static bool ArmDebugCli_HandleTask3(uint8_t token_count)
{
    if (token_count != 1U)
    {
        ArmDebugCli_Send("ERR USAGE: TASK3\r\n");
        return true;
    }

    if (ArmCalibration_IsBusy() ||
        ArmPuzzle_IsBusy() ||
        ArmMotion_IsBusy())
    {
        ArmDebugCli_Send("ERR TASK3 BUSY\r\n");
        return true;
    }

    if (!ArmComm_StartVisionTask3())
    {
        ArmDebugCli_Send("ERR TASK3 USART1_TX_FAILED\r\n");
        return true;
    }

    ArmDebugCli_Send("OK TASK3 TX=3C WAIT_VISION_FRAME\r\n");
    return true;
}

static bool ArmDebugCli_HandleVisionRead(char *tokens[],
                                         uint8_t token_count)
{
    uint16_t task_id;
    char response[112];

    if ((token_count != 2U) ||
        !ArmDebugCli_ParseU16(tokens[1], &task_id) ||
        ((task_id != 1U) && (task_id != 2U) && (task_id != 3U)))
    {
        ArmDebugCli_Send("ERR USAGE: VREAD <1|2|3>\r\n");
        return true;
    }

    if (ArmCalibration_IsBusy() ||
        ArmPuzzle_IsBusy() ||
        ArmMotion_IsBusy())
    {
        ArmDebugCli_Send("ERR VREAD BUSY\r\n");
        return true;
    }

    if (!ArmComm_StartVisionDebug((uint8_t)task_id))
    {
        ArmDebugCli_Send("ERR VREAD USART1_TX_FAILED\r\n");
        return true;
    }

    (void)snprintf(
        response,
        sizeof(response),
        "OK VREAD TASK=%u TX=%s PARSE_ONLY WAIT_VISION_FRAME\r\n",
        (unsigned int)task_id,
        (task_id == 1U) ? "3A" : ((task_id == 2U) ? "3B" : "3C"));
    ArmDebugCli_Send(response);
    return true;
}

static bool ArmDebugCli_HandleVisionSim(char *tokens[],
                                        uint8_t token_count)
{
    uint16_t task_id;
    char response[96];

    if ((token_count != 2U) ||
        !ArmDebugCli_ParseU16(tokens[1], &task_id) ||
        ((task_id != 1U) && (task_id != 2U) && (task_id != 3U)))
    {
        ArmDebugCli_Send("ERR USAGE: VSIM <1|2|3>\r\n");
        return true;
    }

    if (ArmCalibration_IsBusy() ||
        ArmPuzzle_IsBusy() ||
        ArmMotion_IsBusy())
    {
        ArmDebugCli_Send("ERR VSIM BUSY\r\n");
        return true;
    }

    if (!ArmComm_SimulateVisionFrame((uint8_t)task_id))
    {
        ArmDebugCli_Send("ERR VSIM FRAME_REJECTED\r\n");
        return true;
    }

    (void)snprintf(response,
                   sizeof(response),
                   "OK VSIM TASK=%u COUNT=2 ACK=AC EXECUTING\r\n",
                   (unsigned int)task_id);
    ArmDebugCli_Send(response);
    return true;
}

static int8_t ArmDebugCli_HexNibble(char value)
{
    if ((value >= '0') && (value <= '9'))
    {
        return (int8_t)(value - '0');
    }
    if ((value >= 'A') && (value <= 'F'))
    {
        return (int8_t)(value - 'A' + 10);
    }
    return -1;
}

static bool ArmDebugCli_ParseVisionHex(
    const char *text,
    uint8_t frame[ARM_PUZZLE_PACKET_SIZE])
{
    uint16_t index;

    if ((text == NULL) || (frame == NULL) ||
        (strlen(text) != (ARM_PUZZLE_PACKET_SIZE * 2U)))
    {
        return false;
    }

    for (index = 0U; index < ARM_PUZZLE_PACKET_SIZE; ++index)
    {
        int8_t high = ArmDebugCli_HexNibble(text[index * 2U]);
        int8_t low = ArmDebugCli_HexNibble(text[(index * 2U) + 1U]);

        if ((high < 0) || (low < 0))
        {
            return false;
        }

        frame[index] =
            (uint8_t)(((uint8_t)high << 4U) | (uint8_t)low);
    }

    return true;
}

static bool ArmDebugCli_HandleVisionFrame(char *tokens[],
                                          uint8_t token_count)
{
    uint16_t task_id;
    uint8_t frame[ARM_PUZZLE_PACKET_SIZE];
    char response[96];

    if ((token_count != 3U) ||
        !ArmDebugCli_ParseU16(tokens[1], &task_id) ||
        ((task_id != 1U) && (task_id != 2U) && (task_id != 3U)))
    {
        ArmDebugCli_Send(
            "ERR USAGE: VFRAME <1|2|3> <104_HEX_CHARS>\r\n");
        return true;
    }

    if (!ArmDebugCli_ParseVisionHex(tokens[2], frame))
    {
        ArmDebugCli_Send(
            "ERR VFRAME HEX_LENGTH_OR_CHARACTER\r\n");
        return true;
    }

    if (ArmCalibration_IsBusy() ||
        ArmPuzzle_IsBusy() ||
        ArmMotion_IsBusy())
    {
        ArmDebugCli_Send("ERR VFRAME BUSY\r\n");
        return true;
    }

    if (!ArmComm_InjectVisionFrame((uint8_t)task_id,
                                   frame,
                                   ARM_PUZZLE_PACKET_SIZE))
    {
        ArmDebugCli_Send(
            "ERR VFRAME FRAME_HEADER_CHECKSUM_OR_DATA\r\n");
        return true;
    }

    (void)snprintf(response,
                   sizeof(response),
                   "OK VFRAME TASK=%u BYTES=52 ACK=AC EXECUTING\r\n",
                   (unsigned int)task_id);
    ArmDebugCli_Send(response);
    return true;
}

static bool ArmDebugCli_HandleIk(char *tokens[], uint8_t token_count)
{
    int16_t x_x10_mm;
    int16_t y_x10_mm;
    int16_t z_x10_mm;
    int16_t magnet_x10_deg;
    ArmKinematicsSolution solution;
    char response[ARM_DEBUG_CLI_RESPONSE_SIZE];

    if (token_count != 5U)
    {
        ArmDebugCli_Send(
            "ERR USAGE: IK <X_MM> <Y_MM> <Z_MM> <MAGNET_DEG>\r\n");
        return true;
    }

    if (!ArmDebugCli_ParseAngleX10(tokens[1], &x_x10_mm) ||
        !ArmDebugCli_ParseAngleX10(tokens[2], &y_x10_mm) ||
        !ArmDebugCli_ParseAngleX10(tokens[3], &z_x10_mm) ||
        !ArmDebugCli_ParseAngleX10(tokens[4], &magnet_x10_deg))
    {
        ArmDebugCli_Send("ERR IK BAD_PARAMETER\r\n");
        return true;
    }

    if (!ArmKinematics_SolveBoardPose(
            (float)x_x10_mm / 10.0F,
            (float)y_x10_mm / 10.0F,
            (float)z_x10_mm / 10.0F,
            (float)magnet_x10_deg / 10.0F,
            &solution))
    {
        ArmDebugCli_Send("ERR IK IK_UNREACHABLE (14)\r\n");
        return true;
    }

    (void)snprintf(
        response,
        sizeof(response),
        "IK J1=%d.%u J2=%d.%u J3=%d.%u J4=%d.%u J5=%d.%u "
        "BASE=%.2f SHOULDER=%.2f ELBOW=%.2f WRIST=%.2f\r\n",
        (int)(solution.servo_angle_x10[0] / 10),
        (unsigned int)(solution.servo_angle_x10[0] % 10),
        (int)(solution.servo_angle_x10[1] / 10),
        (unsigned int)(solution.servo_angle_x10[1] % 10),
        (int)(solution.servo_angle_x10[2] / 10),
        (unsigned int)(solution.servo_angle_x10[2] % 10),
        (int)(solution.servo_angle_x10[3] / 10),
        (unsigned int)(solution.servo_angle_x10[3] % 10),
        (int)(solution.servo_angle_x10[4] / 10),
        (unsigned int)(solution.servo_angle_x10[4] % 10),
        solution.base_yaw_deg,
        solution.shoulder_deg,
        solution.elbow_deg,
        solution.wrist_deg);
    ArmDebugCli_Send(response);
    return true;
}

static bool ArmDebugCli_HandleGoto(char *tokens[], uint8_t token_count)
{
    int16_t x_x10_mm;
    int16_t y_x10_mm;
    int16_t z_x10_mm;
    int16_t magnet_x10_deg;
    uint16_t duration_ms = ARM_MOTION_DEFAULT_DURATION_MS;
    ArmKinematicsSolution solution;
    ArmResult result;
    char response[ARM_DEBUG_CLI_RESPONSE_SIZE];

    if ((token_count != 5U) && (token_count != 6U))
    {
        ArmDebugCli_Send(
            "ERR USAGE: GOTO <X_MM> <Y_MM> <Z_MM> "
            "<MAGNET_DEG> [TIME_MS]\r\n");
        return true;
    }

    if (!ArmDebugCli_ParseAngleX10(tokens[1], &x_x10_mm) ||
        !ArmDebugCli_ParseAngleX10(tokens[2], &y_x10_mm) ||
        !ArmDebugCli_ParseAngleX10(tokens[3], &z_x10_mm) ||
        !ArmDebugCli_ParseAngleX10(tokens[4], &magnet_x10_deg) ||
        ((token_count == 6U) &&
         !ArmDebugCli_ParseU16(tokens[5], &duration_ms)))
    {
        ArmDebugCli_Send("ERR GOTO BAD_PARAMETER\r\n");
        return true;
    }

    /*
     * GOTO用于手动模拟视觉给出的A4坐标。禁止纸面外和负高度目标，
     * 避免调试输入错误让电磁铁运动到纸面以下。
     */
    if ((x_x10_mm < 0) || (x_x10_mm > 2100) ||
        (y_x10_mm < -1485) || (y_x10_mm > 1485) ||
        (z_x10_mm < 0))
    {
        ArmDebugCli_Send("ERR GOTO OUT_OF_BOARD\r\n");
        return true;
    }

    if (!ArmKinematics_SolveBoardPose(
            (float)x_x10_mm / 10.0F,
            (float)y_x10_mm / 10.0F,
            (float)z_x10_mm / 10.0F,
            (float)magnet_x10_deg / 10.0F,
            &solution))
    {
        ArmDebugCli_Send("ERR GOTO IK_UNREACHABLE (14)\r\n");
        return true;
    }

    result = ArmDebugCli_SubmitMove(solution.servo_angle_x10,
                                    duration_ms,
                                    ARM_CMD_GOTO_BOARD_POSE);
    if (result != ARM_RESULT_OK)
    {
        ArmDebugCli_SendResult("GOTO", result);
        return true;
    }

    (void)snprintf(
        response,
        sizeof(response),
        "OK GOTO X=%.1f Y=%.1f Z=%.1f M=%.1f "
        "J1=%.1f J2=%.1f J3=%.1f J4=%.1f J5=%.1f TIME=%u\r\n",
        (float)x_x10_mm / 10.0F,
        (float)y_x10_mm / 10.0F,
        (float)z_x10_mm / 10.0F,
        (float)magnet_x10_deg / 10.0F,
        (float)solution.servo_angle_x10[0] / 10.0F,
        (float)solution.servo_angle_x10[1] / 10.0F,
        (float)solution.servo_angle_x10[2] / 10.0F,
        (float)solution.servo_angle_x10[3] / 10.0F,
        (float)solution.servo_angle_x10[4] / 10.0F,
        (unsigned int)duration_ms);
    ArmDebugCli_Send(response);
    return true;
}

static bool ArmDebugCli_HandleCalibration(char *tokens[],
                                          uint8_t token_count)
{
    char response[ARM_DEBUG_CLI_RESPONSE_SIZE];

    if (token_count < 2U)
    {
        ArmDebugCli_Send(
            "ERR USAGE: CALIB <CHECK|START|POINT|STATUS|STOP>\r\n");
        return true;
    }

    if (strcmp(tokens[1], "CHECK") == 0)
    {
        uint8_t point;
        uint8_t reachable_mask = 0U;
        ArmResult check_result;

        if (token_count != 2U)
        {
            ArmDebugCli_Send("ERR USAGE: CALIB CHECK\r\n");
            return true;
        }

        check_result = ArmCalibration_Check(&reachable_mask);
        (void)snprintf(
            response,
            sizeof(response),
            "CALIB CHECK HEIGHT=%.1f MASK=0X%02X\r\n",
            ARM_CALIBRATION_HEIGHT_MM,
            (unsigned int)reachable_mask);
        ArmDebugCli_Send(response);

        for (point = 1U;
             point <= ARM_CALIBRATION_POINT_COUNT;
             ++point)
        {
            float x_mm;
            float y_mm;
            ArmKinematicsSolution solution;

            (void)ArmCalibration_GetPoint(point, &x_mm, &y_mm);
            if (ArmCalibration_SolvePoint(point, &solution))
            {
                (void)snprintf(
                    response,
                    sizeof(response),
                    "CALIB P%u X=%.1f Y=%.1f OK "
                    "J1=%.1f J2=%.1f J3=%.1f J4=%.1f J5=%.1f\r\n",
                    (unsigned int)point,
                    x_mm,
                    y_mm,
                    (float)solution.servo_angle_x10[0] / 10.0F,
                    (float)solution.servo_angle_x10[1] / 10.0F,
                    (float)solution.servo_angle_x10[2] / 10.0F,
                    (float)solution.servo_angle_x10[3] / 10.0F,
                    (float)solution.servo_angle_x10[4] / 10.0F);
            }
            else
            {
                (void)snprintf(
                    response,
                    sizeof(response),
                    "CALIB P%u X=%.1f Y=%.1f UNREACHABLE\r\n",
                    (unsigned int)point,
                    x_mm,
                    y_mm);
            }
            ArmDebugCli_Send(response);
        }

        ArmDebugCli_SendResult("CALIB CHECK", check_result);
        return true;
    }

    if (strcmp(tokens[1], "START") == 0)
    {
        ArmResult result;

        if (token_count != 2U)
        {
            ArmDebugCli_Send("ERR USAGE: CALIB START\r\n");
            return true;
        }

        result = ArmCalibration_Start();
        if (result == ARM_RESULT_IK_UNREACHABLE)
        {
            ArmCalibrationStatus status;
            float x_mm = 0.0F;
            float y_mm = 0.0F;

            ArmCalibration_GetStatus(&status);
            (void)ArmCalibration_GetPoint(status.failed_point,
                                          &x_mm,
                                          &y_mm);
            (void)snprintf(
                response,
                sizeof(response),
                "ERR CALIB START IK_UNREACHABLE POINT=%u "
                "X=%.1f Y=%.1f\r\n",
                (unsigned int)status.failed_point,
                x_mm,
                y_mm);
            ArmDebugCli_Send(response);
        }
        else
        {
            ArmDebugCli_SendResult("CALIB START", result);
        }
        return true;
    }

    if (strcmp(tokens[1], "POINT") == 0)
    {
        uint16_t point;
        ArmResult result;

        if ((token_count != 3U) ||
            !ArmDebugCli_ParseU16(tokens[2], &point) ||
            (point < 1U) ||
            (point > ARM_CALIBRATION_POINT_COUNT))
        {
            ArmDebugCli_Send("ERR USAGE: CALIB POINT <1-6>\r\n");
            return true;
        }

        result = ArmCalibration_MoveToPoint((uint8_t)point);
        ArmDebugCli_SendResult("CALIB POINT", result);
        return true;
    }

    if (strcmp(tokens[1], "STATUS") == 0)
    {
        ArmCalibrationStatus status;

        if (token_count != 2U)
        {
            ArmDebugCli_Send("ERR USAGE: CALIB STATUS\r\n");
            return true;
        }

        ArmCalibration_GetStatus(&status);
        (void)snprintf(
            response,
            sizeof(response),
            "CALIB STATUS=%s POINT=%u FAILED=%u RESULT=%s (%u)\r\n",
            ArmCalibration_StateName(status.state),
            (unsigned int)status.current_point,
            (unsigned int)status.failed_point,
            ArmDebugCli_ResultName(status.last_result),
            (unsigned int)status.last_result);
        ArmDebugCli_Send(response);
        return true;
    }

    if (strcmp(tokens[1], "STOP") == 0)
    {
        if (token_count != 2U)
        {
            ArmDebugCli_Send("ERR USAGE: CALIB STOP\r\n");
            return true;
        }

        ArmCalibration_RequestStop();
        ArmDebugCli_Send("OK CALIB STOP\r\n");
        return true;
    }

    ArmDebugCli_Send(
        "ERR USAGE: CALIB <CHECK|START|POINT|STATUS|STOP>\r\n");
    return true;
}

static bool ArmDebugCli_ProcessLine(char *line)
{
    char *tokens[ARM_DEBUG_CLI_MAX_TOKENS];
    uint8_t token_count;

    ArmDebugCli_ToUpper(line);
    token_count = ArmDebugCli_Tokenize(line, tokens);
    if (token_count == 0U)
    {
        return false;
    }

    if (strcmp(tokens[0], "PING") == 0)
    {
        if (token_count == 1U)
        {
            ArmDebugCli_Send("OK PONG\r\n");
        }
        else
        {
            ArmDebugCli_Send("ERR USAGE: PING\r\n");
        }
        return true;
    }

    if (strcmp(tokens[0], "HELP") == 0)
    {
        ArmDebugCli_Send(
            "COMMANDS:\r\n"
            "  PING\r\n"
            "  SERVO <1-5> <ANGLE> [TIME_MS]\r\n"
            "  BASE <ANGLE> [TIME_MS]\r\n"
            "  MOVE <J1> <J2> <J3> <J4> <J5> [TIME_MS]\r\n"
            "  HOME [TIME_MS]\r\n"
            "  MAGNET <ON|OFF>\r\n"
            "  TASK1\r\n"
            "  TASK2\r\n"
            "  TASK3\r\n"
            "  VREAD <1|2|3>\r\n"
            "  VSIM <1|2|3>\r\n"
            "  VFRAME <1|2|3> <104_HEX_CHARS>\r\n"
            "  IK <X_MM> <Y_MM> <Z_MM> <MAGNET_DEG>\r\n"
            "  GOTO <X_MM> <Y_MM> <Z_MM> <MAGNET_DEG> [TIME_MS]\r\n"
            "  CALIB CHECK\r\n"
            "  CALIB START\r\n"
            "  CALIB POINT <1-6>\r\n"
            "  CALIB STATUS\r\n"
            "  CALIB STOP\r\n"
            "  STATUS\r\n"
            "  STOP\r\n");
        return true;
    }

    if (strcmp(tokens[0], "SERVO") == 0)
    {
        return ArmDebugCli_HandleServo(tokens, token_count);
    }
    if (strcmp(tokens[0], "BASE") == 0)
    {
        return ArmDebugCli_HandleBase(tokens, token_count);
    }
    if (strcmp(tokens[0], "MOVE") == 0)
    {
        return ArmDebugCli_HandleMove(tokens, token_count);
    }
    if (strcmp(tokens[0], "HOME") == 0)
    {
        return ArmDebugCli_HandleHome(tokens, token_count);
    }
    if (strcmp(tokens[0], "MAGNET") == 0)
    {
        return ArmDebugCli_HandleMagnet(tokens, token_count);
    }
    if (strcmp(tokens[0], "TASK1") == 0)
    {
        return ArmDebugCli_HandleTask1(token_count);
    }
    if (strcmp(tokens[0], "TASK2") == 0)
    {
        return ArmDebugCli_HandleTask2(token_count);
    }
    if (strcmp(tokens[0], "TASK3") == 0)
    {
        return ArmDebugCli_HandleTask3(token_count);
    }
    if (strcmp(tokens[0], "VREAD") == 0)
    {
        return ArmDebugCli_HandleVisionRead(tokens, token_count);
    }
    if (strcmp(tokens[0], "VSIM") == 0)
    {
        return ArmDebugCli_HandleVisionSim(tokens, token_count);
    }
    if (strcmp(tokens[0], "VFRAME") == 0)
    {
        return ArmDebugCli_HandleVisionFrame(tokens, token_count);
    }
    if (strcmp(tokens[0], "STATUS") == 0)
    {
        return ArmDebugCli_HandleStatus(token_count);
    }
    if (strcmp(tokens[0], "IK") == 0)
    {
        return ArmDebugCli_HandleIk(tokens, token_count);
    }

    if (strcmp(tokens[0], "GOTO") == 0)
    {
        return ArmDebugCli_HandleGoto(tokens, token_count);
    }
    if (strcmp(tokens[0], "CALIB") == 0)
    {
        return ArmDebugCli_HandleCalibration(tokens, token_count);
    }
    if (strcmp(tokens[0], "STOP") == 0)
    {
        if (token_count != 1U)
        {
            ArmDebugCli_Send("ERR USAGE: STOP\r\n");
            return true;
        }
        ArmPuzzle_RequestStop();
        ArmCalibration_RequestStop();
        ArmDebugCli_Send("OK STOP\r\n");
        return true;
    }

    ArmDebugCli_Send("ERR UNKNOWN_COMMAND; SEND HELP\r\n");
    return false;
}

void ArmDebugCli_Init(void)
{
    s_line_length = 0U;
    s_sequence = 0U;
    ArmDebugCli_Send("READY ARM_CLI\r\nSEND HELP FOR COMMANDS\r\n");
}

bool ArmDebugCli_HandleChunk(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    bool handled = false;

    if ((data == NULL) || (length == 0U))
    {
        return false;
    }

    /*
     * USART2 已定义为纯文本口。检测到旧二进制协议或其他控制字节时，
     * 直接给出可读提示，防止把0xAA 0x55误拼进命令缓冲区。
     */
    for (index = 0U; index < length; ++index)
    {
        uint8_t value = data[index];
        if ((value != '\r') &&
            (value != '\n') &&
            (value != '\t') &&
            ((value < 0x20U) || (value > 0x7EU)))
        {
            s_line_length = 0U;
            ArmDebugCli_Send("ERR USART2 TEXT_ONLY; SEND HELP\r\n");
            return false;
        }
    }

    for (index = 0U; index < length; ++index)
    {
        char value = (char)data[index];

        if ((value == '\r') || (value == '\n'))
        {
            if (s_line_length > 0U)
            {
                s_line[s_line_length] = '\0';
                handled = ArmDebugCli_ProcessLine(s_line) || handled;
                s_line_length = 0U;
            }
            continue;
        }

        if (s_line_length >= (ARM_DEBUG_CLI_LINE_SIZE - 1U))
        {
            s_line_length = 0U;
            ArmDebugCli_Send("ERR COMMAND_TOO_LONG\r\n");
            return false;
        }

        s_line[s_line_length++] = value;
    }

    /*
     * 普通短命令在DMA空闲时立即执行，因此VOFA不附加CR/LF也可使用。
     * VFRAME总长113字符，部分串口软件会拆成多个USB/DMA数据块；若
     * 当前内容仍是VFRAME前缀或尚未收满104位HEX，则保留缓冲区等待
     * 下一块，避免前半段报HEX长度错误、后半段报UNKNOWN_COMMAND。
     */
    if (s_line_length > 0U)
    {
        static const char vframe_prefix[] = "VFRAME ";
        const uint16_t vframe_prefix_length =
            (uint16_t)(sizeof(vframe_prefix) - 1U);
        const uint16_t vframe_command_length =
            vframe_prefix_length + 1U + 1U +
            (ARM_PUZZLE_PACKET_SIZE * 2U);
        bool partial_vframe_prefix =
            (s_line_length < vframe_prefix_length) &&
            (memcmp(s_line,
                    vframe_prefix,
                    s_line_length) == 0);
        bool partial_vframe_command =
            (s_line_length >= vframe_prefix_length) &&
            (memcmp(s_line,
                    vframe_prefix,
                    vframe_prefix_length) == 0) &&
            (s_line_length < vframe_command_length);

        if (partial_vframe_prefix || partial_vframe_command)
        {
            return handled;
        }

        s_line[s_line_length] = '\0';
        handled = ArmDebugCli_ProcessLine(s_line) || handled;
        s_line_length = 0U;
    }

    return handled;
}

void ArmDebugCli_HandleEvent(const ArmEvent *event)
{
    char response[96];
    const char *event_name;

    if (event == NULL)
    {
        return;
    }

    if (event->type == ARM_EVENT_SAFETY_ERROR)
    {
        (void)snprintf(response,
                       sizeof(response),
                       "ALARM %s (%u)\r\n",
                       ArmDebugCli_ResultName(event->result),
                       (unsigned int)event->result);
        ArmDebugCli_Send(response);
        return;
    }

    if (event->source_command == ARM_CMD_HOME)
    {
        event_name = "HOME";
    }
    else if (event->source_command == ARM_CMD_PUZZLE_PLAN)
    {
        event_name = "PUZZLE";
    }
    else if (event->source_command == ARM_CMD_GOTO_BOARD_POSE)
    {
        event_name = "GOTO";
    }
    else if (event->source_command == ARM_CMD_CALIBRATION_STEP)
    {
        (void)snprintf(
            response,
            sizeof(response),
            "%s CALIB POINT=%u %s (%u)\r\n",
            (event->result == ARM_RESULT_OK) ? "DONE" : "ERR",
            (unsigned int)event->sequence,
            ArmDebugCli_ResultName(event->result),
            (unsigned int)event->result);
        ArmDebugCli_Send(response);
        return;
    }
    else if (event->source_command == ARM_CMD_CALIBRATION)
    {
        event_name = "CALIB";
    }
    else
    {
        event_name = "MOVE";
    }
    (void)snprintf(response,
                   sizeof(response),
                   "%s %s %s (%u)\r\n",
                   (event->result == ARM_RESULT_OK) ? "DONE" : "ERR",
                   event_name,
                   ArmDebugCli_ResultName(event->result),
                   (unsigned int)event->result);
    ArmDebugCli_Send(response);
}

void ArmDebugCli_ReportVisionPlan(const ArmPuzzlePlan *plan)
{
    char response[192];
    uint8_t order;

    if (plan == NULL)
    {
        return;
    }

    (void)snprintf(response,
                   sizeof(response),
                   "VISION TASK=%u SEQ=%u COUNT=%u PARSE_ONLY NO_MOTION\r\n",
                   (unsigned int)plan->task_id,
                   (unsigned int)plan->sequence,
                   (unsigned int)plan->count);
    ArmDebugCli_Send(response);

    for (order = 0U; order < plan->count; ++order)
    {
        const ArmPuzzlePiece *piece = &plan->pieces[order];

        (void)snprintf(
            response,
            sizeof(response),
            "VISION ORDER=%u ID=%u PICK=(%.1f,%.1f) "
            "PLACE=(%.1f,%.1f) ROTATE=%+.2f\r\n",
            (unsigned int)order,
            (unsigned int)piece->id,
            (float)piece->pick_x_x10_mm / 10.0F,
            (float)piece->pick_y_x10_mm / 10.0F,
            (float)piece->place_x_x10_mm / 10.0F,
            (float)piece->place_y_x10_mm / 10.0F,
            (float)piece->rotate_x100_deg / 100.0F);
        ArmDebugCli_Send(response);
    }

    ArmDebugCli_Send("OK VREAD ACK=AC NO_MOTION\r\n");
}

void ArmDebugCli_ReportVisionReadError(uint8_t task_id,
                                       const char *reason)
{
    char response[112];

    (void)snprintf(response,
                   sizeof(response),
                   "ERR VREAD TASK=%u %s NO_MOTION\r\n",
                   (unsigned int)task_id,
                   (reason != NULL) ? reason : "UNKNOWN");
    ArmDebugCli_Send(response);
}
