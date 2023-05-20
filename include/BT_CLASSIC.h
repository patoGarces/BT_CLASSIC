#ifndef INC_BT_CLASSIC_H
#define INC_BT_CLASSIC_H

#include "stdint.h"
#include "string.h"
#include "esp_spp_api.h"

typedef struct{
    uint32_t header;
    uint32_t header_key;
}header_handler_t;

void bt_init(void);
uint8_t btIsConnected(void);


#endif