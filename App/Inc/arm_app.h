#ifndef ARM_APP_H
#define ARM_APP_H

#include <stdbool.h>

/*
 * 在 HAL_Init/SystemClock_Config/MX_GPIO_Init 之后、启动调度器之前调用。
 * 初始化 PWM、USART1/USART2 DMA、电磁铁与可选急停输入。
 */
bool ArmApp_HardwareInit(void);

/* 在 osKernelInitialize() 之后、osKernelStart() 之前调用。 */
bool ArmApp_RtosInit(void);

#endif /* ARM_APP_H */
