#include "arm_puzzle_protocol.h"

#include <string.h>

#define ARM_PUZZLE_HEADER_1                0xAAU
#define ARM_PUZZLE_HEADER_2                0x55U
#define ARM_PUZZLE_TAIL_1                  0x55U
#define ARM_PUZZLE_TAIL_2                  0xCCU
#define ARM_PUZZLE_RECORDS_OFFSET          5U
#define ARM_PUZZLE_CHECKSUM_OFFSET         49U
#define ARM_PUZZLE_TAIL_OFFSET             50U

static int16_t ArmPuzzleProtocol_ReadI16Be(const uint8_t *data)
{
    uint16_t raw = ((uint16_t)data[0] << 8U) | (uint16_t)data[1];
    return (int16_t)raw;
}

static uint8_t ArmPuzzleProtocol_Checksum(const uint8_t *packet)
{
    uint8_t checksum = 0U;
    uint8_t index;

    for (index = 2U; index <= 48U; ++index)
    {
        checksum = (uint8_t)(checksum + packet[index]);
    }

    return checksum;
}

static uint32_t ArmPuzzleProtocol_Signature(const uint8_t *packet)
{
    uint32_t hash = 2166136261UL;
    uint8_t index;

    /* FNV-1a仅用于识别ACK丢失后的重复计划，不替代协议校验和。 */
    for (index = 0U; index < ARM_PUZZLE_PACKET_SIZE; ++index)
    {
        hash ^= packet[index];
        hash *= 16777619UL;
    }

    return hash;
}

static bool ArmPuzzleProtocol_Decode(const uint8_t *packet,
                                     ArmPuzzlePlan *plan)
{
    uint8_t order;
    uint8_t count;

    if ((packet[0] != ARM_PUZZLE_HEADER_1) ||
        (packet[1] != ARM_PUZZLE_HEADER_2) ||
        (packet[2] != ARM_PUZZLE_PROTOCOL_VERSION) ||
        (packet[4] < ARM_PUZZLE_MIN_PIECE_COUNT) ||
        (packet[4] > ARM_PUZZLE_PIECE_COUNT) ||
        (packet[ARM_PUZZLE_TAIL_OFFSET] != ARM_PUZZLE_TAIL_1) ||
        (packet[ARM_PUZZLE_TAIL_OFFSET + 1U] != ARM_PUZZLE_TAIL_2) ||
        (ArmPuzzleProtocol_Checksum(packet) !=
         packet[ARM_PUZZLE_CHECKSUM_OFFSET]))
    {
        return false;
    }

    count = packet[4];
    plan->task_id = 0U; /* 通信层根据0x3A/0x3B触发来源填写。 */
    plan->sequence = packet[3];
    plan->count = count;
    plan->signature = ArmPuzzleProtocol_Signature(packet);
    (void)memset(plan->pieces, 0, sizeof(plan->pieces));

    for (order = 0U; order < count; ++order)
    {
        const uint8_t *record =
            &packet[ARM_PUZZLE_RECORDS_OFFSET +
                    ((uint16_t)order * ARM_PUZZLE_RECORD_SIZE)];
        ArmPuzzlePiece *piece = &plan->pieces[order];

        piece->id = record[0];
        piece->pick_x_x10_mm = ArmPuzzleProtocol_ReadI16Be(&record[1]);
        piece->pick_y_x10_mm = ArmPuzzleProtocol_ReadI16Be(&record[3]);
        piece->place_x_x10_mm = ArmPuzzleProtocol_ReadI16Be(&record[5]);
        piece->place_y_x10_mm = ArmPuzzleProtocol_ReadI16Be(&record[7]);
        piece->rotate_x100_deg = ArmPuzzleProtocol_ReadI16Be(&record[9]);

        /*
         * 两道题共用完整A4坐标范围。任务2不强制“上半区抓、下半区
         * 放”，ID也只是视觉标识，因此这里只做机械安全边界检查。
         */
        if ((piece->pick_x_x10_mm < 0) ||
            (piece->pick_x_x10_mm > 2100) ||
            (piece->pick_y_x10_mm < -1485) ||
            (piece->pick_y_x10_mm > 1485) ||
            (piece->place_x_x10_mm < 0) ||
            (piece->place_x_x10_mm > 2100) ||
            (piece->place_y_x10_mm < -1485) ||
            (piece->place_y_x10_mm > 1485) ||
            (piece->rotate_x100_deg <= -18000) ||
            (piece->rotate_x100_deg > 18000))
        {
            return false;
        }
    }

    return true;
}

void ArmPuzzleProtocol_Init(ArmPuzzleProtocolParser *parser)
{
    if (parser == NULL)
    {
        return;
    }

    (void)memset(parser, 0, sizeof(*parser));
}

void ArmPuzzleProtocol_Feed(ArmPuzzleProtocolParser *parser,
                            const uint8_t *data,
                            uint16_t length,
                            ArmPuzzlePlanHandler handler,
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

        if (parser->received == 0U)
        {
            if (byte == ARM_PUZZLE_HEADER_1)
            {
                parser->packet[0] = byte;
                parser->received = 1U;
            }
            continue;
        }

        if (parser->received == 1U)
        {
            if (byte == ARM_PUZZLE_HEADER_2)
            {
                parser->packet[1] = byte;
                parser->received = 2U;
            }
            else if (byte == ARM_PUZZLE_HEADER_1)
            {
                parser->packet[0] = byte;
            }
            else
            {
                parser->received = 0U;
            }
            continue;
        }

        parser->packet[parser->received++] = byte;

        if (parser->received == ARM_PUZZLE_PACKET_SIZE)
        {
            ArmPuzzlePlan plan;

            if (ArmPuzzleProtocol_Decode(parser->packet, &plan))
            {
                ++parser->valid_frames;
                handler(&plan, context);
            }
            else
            {
                ++parser->invalid_frames;
            }

            /*
             * 固定52字节包处理完毕后重新搜索AA 55。合法连续包会从
             * 下一个输入字节重新开始；前导残留字节也会被自动跳过。
             */
            parser->received = 0U;
        }
    }
}
