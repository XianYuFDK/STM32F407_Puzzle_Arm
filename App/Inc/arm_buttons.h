#ifndef ARM_BUTTONS_H
#define ARM_BUTTONS_H

#include <stdbool.h>

/*
 * 10路独立按键扫描模块。
 * 默认按键接GND，GPIO内部上拉，按下为低电平。
 */
bool ArmButtons_HardwareInit(void);
bool ArmButtons_RtosInit(void);

#endif /* ARM_BUTTONS_H */
