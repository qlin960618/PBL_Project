#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/crc.h"
#include "system_receiver.h"

#include "gps.h"

//Pins assigment
#define GPS_TX_PIN  22
#define GPS_RX_PIN  23


static const char *TAG = "system_receiver";

static xQueueHandle espnow_recv_event_queue;

/* WiFi should start before using ESPNOW */
static void espnow_wifi_init(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    espnow_recv_event_t evt;
    // espnow_event_recv_info_t *recv_info = &evt.info;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    memcpy(evt.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    evt.data = malloc(len);
    if (evt.data == NULL) {
        ESP_LOGE(TAG, "Malloc receive data fail");
        return;
    }
    memcpy(evt.data, data, len);
    evt.data_len = len;
    if (xQueueSend(espnow_recv_event_queue, &evt, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "Send receive queue fail");
        free(evt.data);
    }
}

static void espnow_routine_task(void)
{

    vTaskDelay(1000 / portTICK_RATE_MS);
    ESP_LOGI(TAG, "Start sending broadcast data");


    espnow_recv_event_t evt;

    while (xQueueReceive(espnow_recv_event_queue, &evt , portMAX_DELAY) == pdTRUE) {

      uint32_t sessionID;
      uint16_t seq_num;
      uint16_t crc;
      gps_data_t gps_data;
      memcpy(&sessionID, evt.data, 4);
      memcpy(&seq_num, evt.data+4, 2);
      memcpy(&crc, evt.data+6, 2);

      gps_data_parser(&gps_data, evt.data+8, crc);

      ESP_LOGI(TAG, "Receive %dth broadcast data from: "MACSTR", len: %d",
               seq_num, MAC2STR(evt.mac_addr), evt.data_len);

      for (int i=0; i<evt.data_len; i++)
      {
        printf("%x ",*(evt.data+i));
      }
      printf("\n");

      printf("ID: %d,\t Fix: %c\n", sessionID, gps_data.fix_status?'Y':'N' );
      printf("Lati: %c %d Deg %.5f M\n", gps_data.lati_ns?'S':'N',
            gps_data.lati_d, gps_data.lati_m);
      printf("Long: %c %d Deg %.5f M\n", gps_data.long_ew?'W':'E',
            gps_data.long_d, gps_data.long_m);
      printf("Time: %d:%d:%.1f\n", gps_data.time_h,gps_data.time_m,
            gps_data.time_s);



      free(evt.data);

    }
}

static esp_err_t espnow_startup_init(void)
{

    espnow_recv_event_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_recv_event_t));
    if (espnow_recv_event_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    // ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    return ESP_OK;
}


void app_main()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    //initialization for espnow comm
    espnow_wifi_init();
    espnow_startup_init();

    //initialization for local GPS module
    gps_init(UART_NUM_1, GPS_RX_PIN, GPS_TX_PIN);


    //periodic task management
    xTaskCreate(espnow_routine_task, "espnow_routine_task", 2048, NULL, 4, NULL);
    gps_watcher_init(2048, 2);


}
