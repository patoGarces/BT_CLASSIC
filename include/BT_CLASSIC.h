#ifndef INC_BT_CLASSIC_H
#define INC_BT_CLASSIC_H

#include "stdint.h"
#include "string.h"
#include "esp_spp_api.h"

void btInit( char* deviceName );
uint8_t btIsConnected(void);


#endif