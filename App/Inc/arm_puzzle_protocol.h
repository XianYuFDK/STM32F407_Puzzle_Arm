#ifndef ARM_PUZZLE_PROTOCOL_H
#define ARM_PUZZLE_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_PUZZLE_PROTOCOL_VERSION       0x01U
/* 任务1/2/3共用固定52字节帧，有效记录数允许2、3或4。 */
#define ARM_PUZZLE_MIN_PIECE_COUNT        2U
#define ARM_PUZZLE_PIECE_COUNT            4U
#define ARM_PUZZLE_RECORD_SIZE            11U
#define ARM_PUZZLE_PACKET_SIZE            52U
#define ARM_PUZZLE_ACK                     0xACU
/* STM32启动视觉任务1：USART1发送单字节0x3A。 */
#define ARM_PUZZLE_TASK1_TRIGGER           0x3AU
/* STM32启动视觉任务2：USART1发送单字节0x3B。 */
#define ARM_PUZZLE_TASK2_TRIGGER           0x3BU
/* STM32启动视觉任务3：USART1发送单字节0x3C。 */
#define ARM_PUZZLE_TASK3_TRIGGER           0x3CU
/* STM32触发视觉校准：USART1发送单字节0x3F。 */
#define ARM_PUZZLE_VISION_CALIB_TRIGGER    0x3FU
/* STM32触发视觉辅助校准：USART1发送单字节0xAD。 */
#define ARM_PUZZLE_VISION_AUX_CALIB_TRIGGER 0xADU

typedef struct
{
    uint8_t id;
    int16_t pick_x_x10_mm;
    int16_t pick_y_x10_mm;
    int16_t place_x_x10_mm;
    int16_t place_y_x10_mm;
    int16_t rotate_x100_deg;
} ArmPuzzlePiece;

typedef struct
{
    /* 1=0x3A任务1，2=0x3B任务2，3=0x3C任务3。 */
    uint8_t task_id;
    uint8_t sequence;
    uint8_t count;
    uint32_t signature;
    ArmPuzzlePiece pieces[ARM_PUZZLE_PIECE_COUNT];
} ArmPuzzlePlan;

typedef struct
{
    uint8_t packet[ARM_PUZZLE_PACKET_SIZE];
    uint8_t received;
    /* 从最近一次TASK1/TASK2触发开始累计，供STATUS诊断。 */
    uint32_t valid_frames;
    uint32_t invalid_frames;
} ArmPuzzleProtocolParser;

typedef void (*ArmPuzzlePlanHandler)(const ArmPuzzlePlan *plan,
                                     void *context);

void ArmPuzzleProtocol_Init(ArmPuzzleProtocolParser *parser);
void ArmPuzzleProtocol_Feed(ArmPuzzleProtocolParser *parser,
                            const uint8_t *data,
                            uint16_t length,
                            ArmPuzzlePlanHandler handler,
                            void *context);

#endif /* ARM_PUZZLE_PROTOCOL_H */
