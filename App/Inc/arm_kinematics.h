#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_config.h"

typedef struct
{
    int16_t servo_angle_x10[ARM_SERVO_COUNT];
    float base_yaw_deg;
    float shoulder_deg;
    float elbow_deg;
    float wrist_deg;
} ArmKinematicsSolution;

/*
 * 输入均为题面A4坐标系：
 *   X向右、Y向下、Z为相对纸面的高度（向上为正），单位mm。
 * magnet_relative_deg是末端旋转轴相对底座平面的转角。
 */
bool ArmKinematics_SolveBoardPose(float board_x_mm,
                                  float board_y_mm,
                                  float board_z_mm,
                                  float magnet_relative_deg,
                                  ArmKinematicsSolution *solution);

bool ArmKinematics_GetBaseYawDeg(float board_x_mm,
                                 float board_y_mm,
                                 float *base_yaw_deg);

float ArmKinematics_NormalizeDeg(float angle_deg);

#endif /* ARM_KINEMATICS_H */
