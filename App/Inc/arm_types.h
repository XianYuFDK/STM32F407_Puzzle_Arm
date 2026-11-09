#ifndef ARM_TYPES_H
#define ARM_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_config.h"

typedef enum
{
    ARM_RESULT_OK = 0,
    ARM_RESULT_BAD_LENGTH = 1,
    ARM_RESULT_BAD_VERSION = 2,
    ARM_RESULT_BAD_CRC = 3,
    ARM_RESULT_BAD_COMMAND = 4,
    ARM_RESULT_ANGLE_LIMIT = 5,
    ARM_RESULT_QUEUE_FULL = 6,
    ARM_RESULT_BUSY = 7,
    ARM_RESULT_HW_ERROR = 8,
    ARM_RESULT_STOPPED = 9,
    ARM_RESULT_ESTOP = 10,
    ARM_RESULT_MAGNET_TIMEOUT = 11,
    ARM_RESULT_COMM_TIMEOUT = 12,
    ARM_RESULT_BAD_PARAMETER = 13,
    ARM_RESULT_IK_UNREACHABLE = 14,
    ARM_RESULT_PLAN_TIMEOUT = 15
} ArmResult;

typedef enum
{
    ARM_COMM_PORT_VISION = 0,
    ARM_COMM_PORT_DEBUG = 1,
    ARM_COMM_PORT_ALL = 2,
    ARM_COMM_PORT_NONE = 3
} ArmCommPort;

typedef struct
{
    uint8_t sequence;
    uint8_t source_command;
    ArmCommPort response_port;
    int16_t target_angle_x10[ARM_SERVO_COUNT];
    uint16_t duration_ms;
    /* 由运动模块入队时填写，用于可靠区分 STOP 前后的命令。 */
    uint32_t stop_epoch;
} ArmMotionCommand;

typedef enum
{
    ARM_EVENT_MOTION_DONE = 0,
    ARM_EVENT_MOTION_STOPPED,
    ARM_EVENT_SAFETY_ERROR
} ArmEventType;

typedef struct
{
    ArmEventType type;
    uint8_t sequence;
    uint8_t source_command;
    ArmCommPort response_port;
    ArmResult result;
} ArmEvent;

#endif /* ARM_TYPES_H */
