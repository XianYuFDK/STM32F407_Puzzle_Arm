#ifndef ARM_PROTOCOL_H
#define ARM_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ARM_PROTOCOL_HEADER_1             0xAAU
#define ARM_PROTOCOL_HEADER_2             0x55U
#define ARM_PROTOCOL_VERSION              0x01U
#define ARM_PROTOCOL_MAX_PAYLOAD          64U
#define ARM_PROTOCOL_MAX_FRAME_SIZE       (ARM_PROTOCOL_MAX_PAYLOAD + 8U)

typedef enum
{
    ARM_CMD_PING = 0x01,
    ARM_CMD_MOVE_JOINTS = 0x10,
    ARM_CMD_MAGNET = 0x11,
    ARM_CMD_HOME = 0x12,
    ARM_CMD_STOP = 0x13,
    ARM_CMD_GET_STATUS = 0x14,
    ARM_CMD_GOTO_BOARD_POSE = 0x15,
    ARM_CMD_PUZZLE_STEP = 0x70,
    ARM_CMD_PUZZLE_PLAN = 0x71,
    ARM_CMD_CALIBRATION_STEP = 0x72,
    ARM_CMD_CALIBRATION = 0x73,
    ARM_CMD_CALIBRATION_TRACE = 0x74,

    ARM_RSP_ACK = 0x80,
    ARM_RSP_DONE = 0x81,
    ARM_RSP_STATUS = 0x82,
    ARM_RSP_ERROR = 0xFF
} ArmProtocolCommand;

typedef struct
{
    uint8_t version;
    uint8_t sequence;
    uint8_t command;
    uint8_t payload_length;
    uint8_t payload[ARM_PROTOCOL_MAX_PAYLOAD];
} ArmProtocolFrame;

typedef enum
{
    ARM_PARSER_WAIT_HEADER_1 = 0,
    ARM_PARSER_WAIT_HEADER_2,
    ARM_PARSER_READ_FRAME
} ArmParserState;

typedef struct
{
    ArmParserState state;
    uint8_t buffer[ARM_PROTOCOL_MAX_FRAME_SIZE];
    uint16_t received;
    uint16_t expected;
} ArmProtocolParser;

typedef void (*ArmProtocolFrameHandler)(const ArmProtocolFrame *frame, void *context);

void ArmProtocol_Init(ArmProtocolParser *parser);
void ArmProtocol_Feed(ArmProtocolParser *parser,
                      const uint8_t *data,
                      uint16_t length,
                      ArmProtocolFrameHandler handler,
                      void *context);
uint16_t ArmProtocol_Crc16Ccitt(const uint8_t *data, uint16_t length);
uint16_t ArmProtocol_Encode(uint8_t sequence,
                           uint8_t command,
                           const uint8_t *payload,
                           uint8_t payload_length,
                           uint8_t *output,
                           uint16_t output_capacity);

#endif /* ARM_PROTOCOL_H */
