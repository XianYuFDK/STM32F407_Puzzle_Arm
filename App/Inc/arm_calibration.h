#ifndef ARM_CALIBRATION_H
#define ARM_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_kinematics.h"
#include "arm_types.h"

typedef enum
{
    ARM_CALIBRATION_STATE_IDLE = 0,
    ARM_CALIBRATION_STATE_QUEUED,
    ARM_CALIBRATION_STATE_RUNNING,
    ARM_CALIBRATION_STATE_COMPLETE,
    ARM_CALIBRATION_STATE_ERROR,
    ARM_CALIBRATION_STATE_STOPPED
} ArmCalibrationState;

typedef struct
{
    ArmCalibrationState state;
    uint8_t current_point;
    uint8_t failed_point;
    ArmResult last_result;
} ArmCalibrationStatus;

/*
 * 创建A4六点坐标校准任务。必须在调度器启动前、运动任务创建后调用。
 */
bool ArmCalibration_RtosInit(void);

/*
 * 检查原点和六个边界点在当前机械参数、舵机零位和限位下是否均可达。
 * reachable_mask的bit0~bit5分别表示P1~P6，1表示可达。
 */
ArmResult ArmCalibration_Check(uint8_t *reachable_mask);

/*
 * 启动A4坐标点检；先到(0,0)，回HOME，再按
 * P1→HOME→P2→HOME→...→P6→HOME逐点检查A4坐标是否正确。
 * 启动前会预解算原点和六个点，任一点不可达则不运动。
 */
ArmResult ArmCalibration_Start(void);

/*
 * 只移动到指定点并停留，方便用尺或指针对单点进行人工测量。
 * point_number取值1~6。
 */
ArmResult ArmCalibration_MoveToPoint(uint8_t point_number);

void ArmCalibration_RequestStop(void);
bool ArmCalibration_IsBusy(void);
void ArmCalibration_GetStatus(ArmCalibrationStatus *status);
const char *ArmCalibration_StateName(ArmCalibrationState state);

/*
 * 获取校准点题面坐标或对应逆解。point_number取值1~6。
 */
bool ArmCalibration_GetPoint(uint8_t point_number,
                             float *board_x_mm,
                             float *board_y_mm);
bool ArmCalibration_SolvePoint(uint8_t point_number,
                               ArmKinematicsSolution *solution);

#endif /* ARM_CALIBRATION_H */
