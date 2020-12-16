#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp32/rom/ets_sys.h"
#include "esp32/rom/crc.h"
#include "system_transmitter.h"

//self defined module
#include "gps.h"

// #define VERBOSE_M

//Pins assigment
#define GPS_TX_PIN  22
#define GPS_RX_PIN  23


static const char *TAG = "system_transmitter";

// gps_data_t _gps_data;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xff,0xff,0xff,0xff,0xff,0xff};
                                      // { 0x7c, 0x9e, 0xbd, 0xf0, 0x9a, 0xd9 };

static void espnow_deinit(espnow_send_param_t *send_param);


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

// espnow initialization
static esp_err_t espnow_startup_init(espnow_send_param_t* send_param)
{

  /* Initialize ESPNOW and register sending and receiving callback function. */
  ESP_ERROR_CHECK( esp_now_init() );
  // ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );

  /* Set primary master key. */
  ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );


  memset(send_param, 0, sizeof(espnow_send_param_t));

  /* Initialize sending parameters. */
  send_param->unicast = false;
  send_param->broadcast = true;
  send_param->state = 0;
  send_param->magic = esp_random();
  send_param->count = CONFIG_ESPNOW_SEND_COUNT;
  send_param->len = (uint16_t)sizeof(gps_data_t)+8;
  // send_param->buffer = malloc(send_param->len);
  // if (send_param->buffer == NULL) {
  //   ESP_LOGE(TAG, "Malloc send buffer fail");
  //   free(send_param);
  //   esp_now_deinit();
  //   return ESP_FAIL;
  // }
  // free(send_param->buffer);
  ESP_LOGI(TAG, "send_param buffer Malloc Successful");
  memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);

  /* Add broadcast peer information to peer list. */
  esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
  if (peer == NULL) {
    ESP_LOGE(TAG, "Malloc peer information fail");
    esp_now_deinit();
    return ESP_FAIL;
  }
  memset(peer, 0, sizeof(esp_now_peer_info_t));
  peer->channel = CONFIG_ESPNOW_CHANNEL;
  peer->ifidx = ESPNOW_WIFI_IF;
  peer->encrypt = false;
  memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
  printf("Adding ESPNOW peer\n");
  esp_err_t err = esp_now_add_peer(peer) ;
  if (err != ESP_OK)
  {
    printf("Error: %s\n", esp_err_to_name(err));
    ESP_ERROR_CHECK( err );
  }
  free(peer);
  if(esp_now_is_peer_exist(broadcast_mac)){
    printf("Added Peer successful\n");
  }

  return ESP_OK;
}


// //prepare GPS data
// void gps_data_prepare(uint8_t *m_payload, uint16_t *m_crc,
//                       gps_data_t *gps_data)
// {
//   #ifdef VERBOSE_M
//     printf("Parsing GPS data to array\n");
//   #endif
//
//   memset(m_payload, 0, sizeof(gps_data_t));
//   *(m_payload+21) = gps_data->time_h;
//   *(m_payload+22) = gps_data->time_m;
//   memcpy(m_payload+23, &(gps_data->time_s), 4);
//
//   if(gps_data->fix_status != 0)
//   {
//     *(m_payload+0) = gps_data->fix_status;
//     *(m_payload+1) = gps_data->lati_d;
//     memcpy(m_payload+2, &(gps_data->lati_m), 8);
//     *(m_payload+10) = gps_data->lati_ns;
//     *(m_payload+11) = gps_data->long_d;
//     memcpy(m_payload+12, &(gps_data->long_m), 8);
//     *(m_payload+20) = gps_data->long_ew;
//     //crc checksum
//     *m_crc = crc16_le(UINT16_MAX, (uint8_t const *)m_payload, sizeof(gps_data_t));
//   }else ////no fix
//   {
//     *m_crc = crc16_le(UINT16_MAX, (uint8_t const *)m_payload, sizeof(gps_data_t));
//   }
// }





static void espnow_routine_task(void *pvParameter)
{

  espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;

  uint16_t seq_num=0;
  uint16_t crc;
  uint32_t sessionID = esp_random();

  vTaskDelay(2000 / portTICK_RATE_MS);
  ESP_LOGI(TAG, "Start sending broadcast data");

  //allocating Buffer
  send_param->buffer = malloc(send_param->len);
  if (send_param->buffer == NULL) {
    ESP_LOGE(TAG, "Malloc send buffer fail");
    free(send_param);
    esp_now_deinit();
    return ESP_FAIL;
  }

  //set dest_mac
  memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);
  esp_err_t err=esp_now_send(send_param->dest_mac,
                             send_param->buffer, send_param->len);
  if (err != ESP_OK) {
    printf("Error: %s\n", esp_err_to_name(err));
    ESP_LOGE(TAG, "Initialization Test Send error");
    espnow_deinit(send_param);
    vTaskDelete(NULL);
  }

  while (true) {
    //halt sender untill data is ready
    while (gps_data_rdy()==false){
      vTaskDelay(100/portTICK_RATE_MS);
    }

    //begin sending of data
    uint8_t data_arr[sizeof(gps_data_t)];
    gps_data_prepare(data_arr, &crc);

    memcpy( (send_param->buffer), &sessionID, 4);
    memcpy( (send_param->buffer)+4, &seq_num, 2);
    memcpy( (send_param->buffer)+6 , &crc, 2);
    memcpy( (send_param->buffer)+8 , data_arr, sizeof(gps_data_t));

    //set dest_mac
    memcpy(send_param->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);

    ESP_LOGI(TAG, "Sending data to "MACSTR"", MAC2STR(send_param->dest_mac));
    #ifdef VERBOSE_M
      printf("Length: %d\n Data:",send_param->len);
      for (int i=0; i<send_param->len; i++)
      {
        printf("%x ",*((send_param->buffer)+i));
      }
      printf("\n");
    #endif

    //sending
    esp_err_t err=esp_now_send(send_param->dest_mac,
                               send_param->buffer, send_param->len);
    if (err != ESP_OK) {
      printf("Error: %s\n", esp_err_to_name(err));
      ESP_LOGE(TAG, "Send error");
      espnow_deinit(send_param);
      vTaskDelete(NULL);
    }
    //increment sequence counter
    seq_num ++;

  }
}

static void espnow_deinit(espnow_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    esp_now_deinit();
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

    espnow_wifi_init();
    //intiialize send_param
    espnow_send_param_t send_param;

    espnow_startup_init(&send_param);

    //gps Initialization
    gps_init(UART_NUM_1, GPS_RX_PIN, GPS_TX_PIN);

    //begin espnow loop
    xTaskCreate(espnow_routine_task, "espnow_routine_task", 2048*2, &send_param, 4, NULL);

    //begin loop for gps
    gps_watcher_init(2048, 2);

}
