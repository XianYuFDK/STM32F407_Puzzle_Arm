#ifndef BSP_MAGNET_H
#define BSP_MAGNET_H

#include <stdbool.h>
#include <stdint.h>

void BspMagnet_Init(void);
void BspMagnet_Set(bool enabled);
bool BspMagnet_IsOn(void);
uint32_t BspMagnet_GetOnSinceTick(void);

#endif /* BSP_MAGNET_H */
