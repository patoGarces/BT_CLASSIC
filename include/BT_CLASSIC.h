#ifndef INC_BT_CLASSIC_H
#define INC_BT_CLASSIC_H

#include "stdint.h"
#include "esp_spp_api.h"

typedef struct{
    uint16_t bat_voltage;
    uint16_t temp_uc_control;
    uint16_t temp_uc_main;
    uint16_t speedR;
    uint16_t speedL;
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
    uint16_t centerAngle;
    uint16_t P;
    uint16_t I;
    uint16_t D;
    uint8_t orden_code;
    uint8_t error_code;
}tx_bt_app_t;

typedef struct{
    uint16_t P;
    uint16_t I;
    uint16_t D;
    uint8_t enable;
    uint8_t orden_code;
    uint8_t error_code;
}rx_bt_app_t;

void bt_init(void);
void btSendData(float x,float y, uint16_t motores);
void btReceiveData(esp_spp_cb_param_t *param);

uint8_t btIsConnected(void);
void btSendAngle(float ejeX,float ejeY,float ejeZ);

#endif