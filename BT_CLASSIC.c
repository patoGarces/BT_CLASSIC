#include <stdio.h>
#include "BT_CLASSIC.h"

#include "driver/gpio.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "spp_task.h"
#include "sys/unistd.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXCAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"

 #define SPP_DATA_LEN 100

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static void spp_read_handle(void * param);

uint32_t handleSpp;
uint8_t btConnected=false;                                        //guardo el estado de conexion

 typedef struct{
    uint32_t header;
    uint32_t kp;
    uint32_t ki;
    uint32_t kd;
    uint32_t checksum;
} pidSettings_t;


QueueHandle_t queueReceive;

static void spp_read_handle(void * param)
{
    queueReceive = xQueueCreate(1, sizeof(pidSettings_t));

    pidSettings_t pidSettings;
 
    pidSettings.header = 0;
    pidSettings.kp = 0;
    pidSettings.ki = 0;
    pidSettings.kd = 0;

    do {

        if( xQueueReceive(queueReceive,
                         &pidSettings,
                         ( TickType_t ) 100 ) == pdPASS ){
                            printf("Paquete recibido\n");
                            esp_log_buffer_hex("ENQUEUE RECEIVE:", &pidSettings, sizeof(pidSettings));
                         

            // ESP_LOGI(SPP_TAG, "fd = %d data_len = %d", fd, size);
            // esp_log_buffer_hex(SPP_TAG, spp_data, size);

            // memcpy(&pidSettings,spp_data,sizeof(pidSettings));

            // esp_log_buffer_hex(SPP_TAG, &pidSettings, size);

            printf("KP = %ld, KI = %ld, KD = %ld, checksum: %ld\n", pidSettings.kp,pidSettings.ki,pidSettings.kd,pidSettings.checksum);

            if(pidSettings.header == 0xABC0){
                printf("HEader detectado! \n");
            }

            if(pidSettings.checksum == (pidSettings.header ^ pidSettings.kp ^ pidSettings.ki ^ pidSettings.kd)){
                printf("Paquete valido!\n");
            }

        }
    } while (1);

    spp_wr_task_shut_down();
}


static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{

    handleSpp = param->open.handle;

    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(EXCAMPLE_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
        btConnected = false;
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        // ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len < 1023) {
            // snprintf(buf, (size_t)param->data_ind.len, (char *)param->data_ind.data);
            // printf("%s\n", buf);
            // sprintf(spp_data, "Received characters: %d\n", param->data_ind.len);
            // esp_spp_write(param->write.handle, strlen(spp_data), (uint8_t *)spp_data);

            pidSettings_t pidSettings;
            memcpy(&pidSettings,param->data_ind.data,sizeof(pidSettings));

            xQueueSend(queueReceive,( void * ) &pidSettings, 0);
            // esp_log_buffer_hex("ESP_SPP_DATA_IND_EVT",param->data_ind.data,param->data_ind.len);
        }
        else {
            esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
        }
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        btConnected = true;
        spp_wr_task_start_up(spp_read_handle, param->srv_open.fd);
        break;
    default:
        break;
    }
}


void bt_init(void){
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );


    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    if (esp_bluedroid_init() != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed\n", __func__);
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed\n", __func__);
        return;
    }

    if (esp_spp_register_callback(esp_spp_cb) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed\n", __func__);
        return;
    }

    if (esp_spp_init(esp_spp_mode) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed\n", __func__);
        return;
    }

     printf("Bluetooth iniciado exitosamente\n");
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


// // /* esta funcion se llama desde el handler de eventos de bt spp, en param incluye la informacion recibida, asi como el handle y la longitud */
// void btReceiveData(esp_spp_cb_param_t *param){
//     char spp_data[100];

// //     // if(strstr((const char*)param->data_ind.data,"LEDON") != NULL){
// //     //     gpio_set_level(LED2,1);
// //     //     printf("LED ENCENDIDO\n");
// //     // }
// //     // if(strstr((const char*)param->data_ind.data,"LEDOFF") != NULL){
// //     //     gpio_set_level(LED2,0);
// //     //     printf("LED APAGADO\n");

//     // }
//     /* respondo */
//     sprintf(spp_data, "Comando recibido: %s\n", param->data_ind.data);
//     esp_spp_write(param->data_ind.handle, strlen(spp_data), (uint8_t *)spp_data);

// }