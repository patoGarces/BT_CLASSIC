#include <stdio.h>
#include "BT_CLASSIC.h"

#include "driver/gpio.h"
#include "esp_bt.h"                                         // funciones basicas de bt
#include "esp_log.h"                                        // para poder realizar logs
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "time.h"
#include "sys/time.h"
#include "string.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "HolaMundoBT"
#define EXAMPLE_DEVICE_NAME "HolaMundoBT1"                  // nombre del dispositivo

#define mode_bt ESP_BT_MODE_CLASSIC_BT                      // modo clasico: MOODO_BT_MODE_CLASSIC_BT,  modo BLE: ESP_BT_MODE_BLE, modo dual: ESP_BT_MODE_BTDM

static struct timeval time_old;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

uint32_t handleSpp;
uint8_t btConnected=false;                                        //guardo el estado de conexion

void handler_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

void handler_spp_cb(esp_spp_cb_event_t event,esp_spp_cb_param_t *param){

    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");                                  // se llama cuando el spp inicia
            esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:                                            // se llama cuando se completa el descubrimiento(?)
            ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
            break;
        case ESP_SPP_OPEN_EVT:                                                      // se llama cuando se abre la conexion con el cliente spp
            ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
            
            break;
        case ESP_SPP_CLOSE_EVT:                                                     // se llama cuando se cierra la conexion con el cliente spp
            ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
            btConnected=false;
            break;
        case ESP_SPP_START_EVT:                                                     // se llama cuando se inicializa el servidor SPP
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            break;
        case ESP_SPP_CL_INIT_EVT:                                                   // se llama cuando el cliente inicio el servidor SPP
            ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
            break;
        case ESP_SPP_DATA_IND_EVT:                                                  // se llama cuando se recibe informacion de la conexion spp
            ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                    param->data_ind.len, param->data_ind.handle);
            esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
            ESP_LOG_BUFFER_CHAR("",param->data_ind.data,param->data_ind.len);

            btReceiveData(param);
            break;
        case ESP_SPP_CONG_EVT:                                                      // se llama cuando cambia el estado de congestion, solo para ESP_SPP_MODE_CB
            ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
            break;
        case ESP_SPP_WRITE_EVT:                                                     // se llama cuando se completa la escritura
            // ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
            break;
        case ESP_SPP_SRV_OPEN_EVT:                                                  // se llama cuando se abre el servidor SPP
            ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
            handleSpp = param->srv_open.handle;
            ESP_LOGE(SPP_TAG,"Handle hallado: %d",handleSpp);
            gettimeofday(&time_old, NULL);  
            btConnected=true;           
            break;
        case ESP_SPP_SRV_STOP_EVT:                                                  // se llama cuando el servidor SPP se para                 
            ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
            break;
        case ESP_SPP_UNINIT_EVT:                                                    // se llama cuando el servidor SPP no esta inicializado
            ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
            break;
        default:
            break;
    }

}

esp_err_t bt_init(void){
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));                                        // libero memoria antes de iniciar

    esp_bt_controller_config_t  bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();                               //inicio a variable bt_cfg

    if((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK){                                                  // inicializo el controlador bt
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    if((ret = esp_bt_controller_enable(mode_bt)) != ESP_OK){                                                // habilito controlador bluetooth
        ESP_LOGE(SPP_TAG, "Error habilitando bluetooth: %s , funcion: %s", esp_err_to_name(ret), __func__);
        return ESP_FAIL;
    }

    if((ret = esp_bluedroid_init()) != ESP_OK){                                                             // inicializo bluedroid
        ESP_LOGE(SPP_TAG,"Error inicializando bluedroid %s , funcion %s", esp_err_to_name(ret),__func__);
        return ESP_FAIL;
    }

    if((ret = esp_bluedroid_enable()) != ESP_OK){                                                           // habilito bluedroid
        ESP_LOGE(SPP_TAG,"Error habilitando bluedroid: %s , funcion %s",esp_err_to_name(ret),__func__);
        return ESP_FAIL;
    }

    // if((ret = esp_bt_gap_register_callback(handler_gap_cb)) != ESP_OK){                                      // registro callback de que?
    //     ESP_LOGE(SPP_TAG,"Error registrando el callback para no se que %s, funcion: %s",esp_err_to_name(ret),__func__);
    //     return ESP_FAIL;
    // }

    if((ret = esp_spp_register_callback(handler_spp_cb)) != ESP_OK){
        ESP_LOGE(SPP_TAG,"Error registrando callback para perfil SPP: %s",esp_err_to_name(ret));
        return ESP_FAIL;
    }

    if((ret = esp_spp_init(esp_spp_mode)) != ESP_OK){
        ESP_LOGE(SPP_TAG,"Error iniciando el modo SPP: %s",esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /* para habilitar el SSP*/
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_OUT;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    return ESP_OK;
}

void btSendData(float x,float y, uint16_t motores){
    char spp_data[256];
    sprintf(spp_data, "X: %f Y: %f, Motores: %d\n", x,y,motores);
    esp_spp_write(handleSpp, strlen(spp_data), (uint8_t *)spp_data);
}

void btSendAngle(float ejeX,float ejeY,float ejeZ){
    char spp_data[256];
    sprintf(spp_data,"Angle X: %f\fAngle Y: %f\fAngle Z: %f\f\n", ejeX, ejeY, ejeZ);
    esp_spp_write(handleSpp, strlen(spp_data), (uint8_t *)spp_data);
}

uint8_t btIsConnected(void){
    return btConnected;
}


/* esta funcion se llama desde el handler de eventos de bt spp, en param incluye la informacion recibida, asi como el handle y la longitud */
void btReceiveData(esp_spp_cb_param_t *param){
    char spp_data[100];

    // if(strstr((const char*)param->data_ind.data,"LEDON") != NULL){
    //     gpio_set_level(LED2,1);
    //     printf("LED ENCENDIDO\n");
    // }
    // if(strstr((const char*)param->data_ind.data,"LEDOFF") != NULL){
    //     gpio_set_level(LED2,0);
    //     printf("LED APAGADO\n");

    // }
    /* respondo */
    sprintf(spp_data, "Comando recibido: %s\n", param->data_ind.data);
    esp_spp_write(param->data_ind.handle, strlen(spp_data), (uint8_t *)spp_data);

}