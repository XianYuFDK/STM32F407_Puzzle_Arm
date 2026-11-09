#ifndef BSP_SAFETY_IO_H
#define BSP_SAFETY_IO_H

#include <stdbool.h>

void BspSafetyIo_Init(void);
bool BspSafetyIo_IsEmergencyStopActive(void);

#endif /* BSP_SAFETY_IO_H */
