#ifndef INC_BT_CLASSIC_H
#define INC_BT_CLASSIC_H

#include "stdint.h"
#include "string.h"
#include "esp_spp_api.h"

void bt_init(void);
void btSendData(float x,float y, uint16_t motores);
void btReceiveData(esp_spp_cb_param_t *param);

uint8_t btIsConnected(void);
void btSendAngle(float ejeX,float ejeY,float ejeZ);

#endif