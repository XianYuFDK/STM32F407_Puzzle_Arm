#ifndef ARM_PUZZLE_H
#define ARM_PUZZLE_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_puzzle_protocol.h"
#include "arm_types.h"

typedef enum
{
    ARM_PUZZLE_STATE_IDLE = 0,
    ARM_PUZZLE_STATE_QUEUED,
    ARM_PUZZLE_STATE_RUNNING,
    ARM_PUZZLE_STATE_COMPLETE,
    ARM_PUZZLE_STATE_ERROR,
    ARM_PUZZLE_STATE_STOPPED
} ArmPuzzleState;

typedef enum
{
    ARM_PUZZLE_SUBMIT_ACCEPTED = 0,
    ARM_PUZZLE_SUBMIT_DUPLICATE,
    ARM_PUZZLE_SUBMIT_BUSY,
    ARM_PUZZLE_SUBMIT_INVALID
} ArmPuzzleSubmitResult;

typedef struct
{
    ArmPuzzleState state;
    uint8_t sequence;
    uint8_t current_piece;
    ArmResult last_result;
} ArmPuzzleStatus;

bool ArmPuzzle_RtosInit(void);
ArmPuzzleSubmitResult ArmPuzzle_SubmitPlan(const ArmPuzzlePlan *plan);
void ArmPuzzle_RequestStop(void);
bool ArmPuzzle_IsBusy(void);
void ArmPuzzle_GetStatus(ArmPuzzleStatus *status);
const char *ArmPuzzle_StateName(ArmPuzzleState state);

#endif /* ARM_PUZZLE_H */
