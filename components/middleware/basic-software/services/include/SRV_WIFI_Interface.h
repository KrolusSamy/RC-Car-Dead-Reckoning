#ifndef SRV_WIFI_INTERFACE_H_
#define SRV_WIFI_INTERFACE_H_

#include "Error_States.h"
#include <stdbool.h>

ErrorState_TypeDef SRV_WIFI_Init(void);
bool SRV_WIFI_IsConnected(void);

#endif /* SRV_WIFI_INTERFACE_H_ */
