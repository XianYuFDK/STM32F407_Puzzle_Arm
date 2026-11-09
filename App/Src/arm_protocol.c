#include "arm_protocol.h"

#include <string.h>

uint16_t ArmProtocol_Crc16Ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    if (data == NULL)
    {
        return crc;
    }

    for (index = 0U; index < length; ++index)
    {
        crc ^= (uint16_t)data[index] << 8U;

        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

void ArmProtocol_Init(ArmProtocolParser *parser)
{
    if (parser == NULL)
    {
        return;
    }

    (void)memset(parser, 0, sizeof(*parser));
    parser->state = ARM_PARSER_WAIT_HEADER_1;
}

static void ArmProtocol_ResetParser(ArmProtocolParser *parser)
{
    parser->state = ARM_PARSER_WAIT_HEADER_1;
    parser->received = 0U;
    parser->expected = 0U;
}

static void ArmProtocol_DeliverFrame(const ArmProtocolParser *parser,
                                     ArmProtocolFrameHandler handler,
                                     void *context)
{
    ArmProtocolFrame frame;
    uint8_t payload_length = parser->buffer[5];

    frame.version = parser->buffer[2];
    frame.sequence = parser->buffer[3];
    frame.command = parser->buffer[4];
    frame.payload_length = payload_length;

    if (payload_length > 0U)
    {
        (void)memcpy(frame.payload, &parser->buffer[6], payload_length);
    }

    handler(&frame, context);
}

void ArmProtocol_Feed(ArmProtocolParser *parser,
                      const uint8_t *data,
                      uint16_t length,
                      ArmProtocolFrameHandler handler,
                      void *context)
{
    uint16_t index;

    if ((parser == NULL) || (data == NULL) || (handler == NULL))
    {
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        uint8_t byte = data[index];

        switch (parser->state)
        {
            case ARM_PARSER_WAIT_HEADER_1:
                if (byte == ARM_PROTOCOL_HEADER_1)
                {
                    parser->buffer[0] = byte;
                    parser->received = 1U;
                    parser->state = ARM_PARSER_WAIT_HEADER_2;
                }
                break;

            case ARM_PARSER_WAIT_HEADER_2:
                if (byte == ARM_PROTOCOL_HEADER_2)
                {
                    parser->buffer[1] = byte;
                    parser->received = 2U;
                    parser->state = ARM_PARSER_READ_FRAME;
                }
                else if (byte == ARM_PROTOCOL_HEADER_1)
                {
                    /* 连续 AA 时，将后一个 AA 当作新帧起点。 */
                    parser->buffer[0] = byte;
                    parser->received = 1U;
                }
                else
                {
                    ArmProtocol_ResetParser(parser);
                }
                break;

            case ARM_PARSER_READ_FRAME:
                if (parser->received >= ARM_PROTOCOL_MAX_FRAME_SIZE)
                {
                    ArmProtocol_ResetParser(parser);
                    break;
                }

                parser->buffer[parser->received++] = byte;

                /* 收到固定头部中的 payload_length 后确定整帧长度。 */
                if (parser->received == 6U)
                {
                    uint8_t payload_length = parser->buffer[5];

                    if (payload_length > ARM_PROTOCOL_MAX_PAYLOAD)
                    {
                        ArmProtocol_ResetParser(parser);
                        break;
                    }

                    parser->expected = (uint16_t)payload_length + 8U;
                }

                if ((parser->expected > 0U) &&
                    (parser->received == parser->expected))
                {
                    uint8_t payload_length = parser->buffer[5];
                    uint16_t calculated_crc =
                        ArmProtocol_Crc16Ccitt(&parser->buffer[2],
                                             (uint16_t)payload_length + 4U);
                    uint16_t received_crc =
                        (uint16_t)parser->buffer[6U + payload_length] |
                        ((uint16_t)parser->buffer[7U + payload_length] << 8U);

                    if (calculated_crc == received_crc)
                    {
                        ArmProtocol_DeliverFrame(parser, handler, context);
                    }

                    ArmProtocol_ResetParser(parser);
                }
                break;

            default:
                ArmProtocol_ResetParser(parser);
                break;
        }
    }
}

uint16_t ArmProtocol_Encode(uint8_t sequence,
                           uint8_t command,
                           const uint8_t *payload,
                           uint8_t payload_length,
                           uint8_t *output,
                           uint16_t output_capacity)
{
    uint16_t frame_length = (uint16_t)payload_length + 8U;
    uint16_t crc;

    if ((output == NULL) ||
        (payload_length > ARM_PROTOCOL_MAX_PAYLOAD) ||
        (output_capacity < frame_length) ||
        ((payload_length > 0U) && (payload == NULL)))
    {
        return 0U;
    }

    output[0] = ARM_PROTOCOL_HEADER_1;
    output[1] = ARM_PROTOCOL_HEADER_2;
    output[2] = ARM_PROTOCOL_VERSION;
    output[3] = sequence;
    output[4] = command;
    output[5] = payload_length;

    if (payload_length > 0U)
    {
        (void)memcpy(&output[6], payload, payload_length);
    }

    crc = ArmProtocol_Crc16Ccitt(&output[2], (uint16_t)payload_length + 4U);
    output[6U + payload_length] = (uint8_t)(crc & 0xFFU);
    output[7U + payload_length] = (uint8_t)(crc >> 8U);

    return frame_length;
}
