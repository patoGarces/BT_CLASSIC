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
#include "sys/unistd.h"
#include "freertos/stream_buffer.h"

#include "../../../include/comms.h"

#define BT_CORE 0

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define BT_DEVICE_NAME "ESP32_BRAZO_ROBOT"

#define STREAM_BUFFER_SIZE              512
#define STREAM_BUFFER_LENGTH_TRIGGER    3

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

StreamBufferHandle_t xStreamBufferReceiver;
StreamBufferHandle_t xStreamBufferSender;

uint32_t handleSpp;
uint8_t btConnected=false;                                        //guardo el estado de conexion

QueueHandle_t queueSend;

static void handlerEnqueueSender(void *pvParameters);

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    handleSpp = param->open.handle;

    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(BT_DEVICE_NAME);
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
        if (param->data_ind.len < 1023) {
                /* Intenta escribir en el buffer */
            if (xStreamBufferSend(xStreamBufferReceiver, param->data_ind.data, param->data_ind.len, 1) != pdPASS) {
                /* Manejar el caso en el que el buffer está lleno y no se pueden enviar datos */
                // TODO: manejar caso de buffer lleno
            }
        }
        else {
            esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
        }
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        // ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        btConnected = true;
        spp_wr_task_start_up();
        xTaskCreatePinnedToCore(handlerEnqueueSender,"queue sender manager",2048,NULL,5,NULL,BT_CORE);
        
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

    esp_spp_cfg_t configSpp = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = false,  
    };

    if (esp_spp_enhanced_init(&configSpp) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed\n", __func__);
        return;
    }

     printf("Bluetooth iniciado exitosamente\n");
     xStreamBufferSender = xStreamBufferCreate( STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER );
     xStreamBufferReceiver = xStreamBufferCreate( STREAM_BUFFER_SIZE, STREAM_BUFFER_LENGTH_TRIGGER );
}

static void handlerEnqueueSender(void *pvParameters){

    char received_data[100];
    
    while(btConnected){

        BaseType_t bytes_received = xStreamBufferReceive(xStreamBufferSender, received_data, sizeof(received_data) - 1, pdMS_TO_TICKS(1));

        if (bytes_received > 0 && btIsConnected()) {
            // received_data[bytes_received] = '\0';                       // Asegurar que la cadena esté terminada correctamente
            // printf("Sender: %ssize: %d\n", received_data,bytes_received);
            esp_spp_write(handleSpp,bytes_received,(uint8_t *)&received_data);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}



uint8_t btIsConnected(void){
    return btConnected;
}
