#ifndef SRV_UDP_INTERFACE_H_
#define SRV_UDP_INTERFACE_H_

#include "Error_States.h"
#include <stdint.h>

ErrorState_TypeDef SRV_UDP_Init(void);
ErrorState_TypeDef SRV_UDP_Send(const char* payload);
ErrorState_TypeDef SRV_UDP_Receive(char* buffer, uint16_t max_len);

#endif /* SRV_UDP_INTERFACE_H_ */