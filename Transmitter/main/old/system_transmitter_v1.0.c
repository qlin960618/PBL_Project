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
// #include "gps.h"

// #define VERBOSE_M

//Pins assigment
#define GPS_TX_PIN  22
#define GPS_RX_PIN  23
#define UART_RTS  (UART_PIN_NO_CHANGE)
#define UART_CTS  (UART_PIN_NO_CHANGE)

//UART Parameter
#define BUF_SIZE (1024)


static const char *TAG = "system_transmitter";

bool _gps_data_rdy = false;
esp_gps_data_t _gps_data;

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


  /* Initialize sending parameters. */

  send_param->unicast = false;
  send_param->broadcast = true;
  send_param->state = 0;
  send_param->magic = esp_random();
  send_param->count = CONFIG_ESPNOW_SEND_COUNT;
  // send_param->buffer = malloc(sizeof(esp_gps_data_t)+4);
  send_param->len = (uint16_t)sizeof(send_param->buffer);
  if (send_param->buffer == NULL) {
    ESP_LOGE(TAG, "Malloc send buffer fail");
    free(send_param);
    esp_now_deinit();
    return ESP_FAIL;
  }
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


void gps_uart_init()
{
  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(UART_NUM_1, &uart_config);
  uart_set_pin(UART_NUM_1, GPS_TX_PIN, GPS_RX_PIN, UART_RTS, UART_CTS);
  esp_err_t err = uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
  printf("%s\n",esp_err_to_name(err));
  printf("uart Initialize complete\n");


}



//prepare GPS data
void gps_data_prepare(uint8_t *m_payload, uint16_t *m_crc,
                      esp_gps_data_t *gps_data)
{
  #ifdef VERBOSE_M
    printf("Parsing GPS data to array\n");
  #endif

  if(gps_data->fix_status != 0)
  {
    *(m_payload+0) = gps_data->fix_status;
    *(m_payload+1) = gps_data->lati_d;
    memcpy(m_payload+2, &(gps_data->lati_m), 8);
    *(m_payload+10) = gps_data->lati_ns;
    *(m_payload+11) = gps_data->long_d;
    memcpy(m_payload+12, &(gps_data->long_m), 8);
    *(m_payload+20) = gps_data->long_ew;
    *(m_payload+21) = gps_data->time_h;
    *(m_payload+22) = gps_data->time_m;
    memcpy(m_payload+23, &(gps_data->time_s), 4);
    //crc checksum
    *m_crc = crc16_le(UINT16_MAX, (uint8_t const *)m_payload, sizeof(esp_gps_data_t));
  }else ////no fix
  {
    memset(m_payload, 0, sizeof(esp_gps_data_t));
    *m_crc = crc16_le(UINT16_MAX, (uint8_t const *)m_payload, sizeof(esp_gps_data_t));
  }
}





static void espnow_routine_task(void *pvParameter)
{

  espnow_send_param_t *send_param = (espnow_send_param_t *)pvParameter;

  uint16_t seq_num=0;
  uint16_t crc;
  uint32_t sessionID = esp_random();

  vTaskDelay(2000 / portTICK_RATE_MS);
  ESP_LOGI(TAG, "Start sending broadcast data");

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

    while (!_gps_data_rdy){
      vTaskDelay(1000/portTICK_RATE_MS);
    }

/*
    //preparing fake data
    _gps_data.fix_status=1;
    _gps_data.lati_d=11;
    _gps_data.lati_d=22;       //Latitude in Degree
    _gps_data.lati_m=23.1111;       //Latitude in minutes
    _gps_data.lati_ns=0;      //N or S == 0 or 1
    _gps_data.long_d=23;       //Longtitude in Degree
    _gps_data.long_m=3.22222;       //Longtitude in Minutes
    _gps_data.long_ew=1;      //E or W == 0 or 1
    _gps_data.time_h=12;       //UTC time H
    _gps_data.time_m=12;       //UTC time M
    _gps_data.time_s=12;
*/

    //begin sending of data
    uint8_t data_arr[sizeof(_gps_data)];
    gps_data_prepare(data_arr, &crc, &_gps_data);

    memcpy( (send_param->buffer), &sessionID, 4);
    memcpy( (send_param->buffer)+4, &seq_num, 2);
    memcpy( (send_param->buffer)+6 , &crc, 2);
    memcpy( (send_param->buffer)+8 , data_arr, sizeof(_gps_data));

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
    }else{
      _gps_data_rdy=false;
    }

    //increment sequence counter
    seq_num ++;

  }
}

static void gps_routine_task(void)
{
  printf("Begin reading GPS data\n");

  uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

  uart_flush(UART_NUM_1);
  while(1)
  {
    int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE,
                              150 / portTICK_RATE_MS);

    if(len>0){
      //new page received
      uint16_t d_frame_s[25];
      int frame_cnt=0;
      memset(d_frame_s,0,24);

      // printf("Length: %d\n", len);
      //count frames
      for(int i=0; i<len; i++){
        if(*(data+i)=='$'){
          d_frame_s[frame_cnt++]=i;
        }
        // printf("%c",*(data+i));
      }
      // printf("Total %d Frame\n",frame_cnt);

      //loop through each frame
      for(int i=0; i<frame_cnt; i++)
      {
        uint16_t s_pos=d_frame_s[i];
        uint16_t e_pos=(d_frame_s[i+1]==0?len:d_frame_s[i+1])-2;
        uint32_t header = (*(data+s_pos+3))<<16
                         |(*(data+s_pos+4))<<8
                         |(*(data+s_pos+5));
        // printf("%c%c%c-%x, ",*(data+s_pos+3),*(data+s_pos+4),*(data+s_pos+5),header);

        uint8_t param_cnt=0;
        uint16_t param_pos=s_pos+7;

        for(int j=s_pos+6; j<=e_pos; j++){
          //find  "," index
          if(*(data+j) == ','){
            param_cnt++;
            param_pos=j;
          }

          switch (header){
            case 0x475341: {  //case for GSA
              // printf("GSA: ");
              // get Fix
              if(param_cnt==2 && param_pos==j && *(data+j+1)!=','){
                  _gps_data.fix_status=*(data+j+1)-'1';
                  printf("Able to read fix val: %d\n",_gps_data.fix_status);
              }
              // printf("%c", *(data+j));
              break;
            }
            case 0x474c4c: {  //case for GLL
              // get Latitude
              if(param_cnt==1 && param_pos==j && *(data+j+1)!=','){
                _gps_data.lati_d=((*(data+j+1)-'0')*10+(*(data+j+2)-'0'));
                _gps_data.lati_m=((*(data+j+3)-'0')*10+(*(data+j+4)-'0'));
                int m_dec=0;
                for(int k=0; k<5; k++)
                  m_dec=m_dec*10+(*(data+j+6+k)-'0');
                _gps_data.lati_m+=m_dec/100000.0;
                // printf("Able to read latitude: %d %.5f\n",_gps_data.lati_d,
                //         _gps_data.lati_m);
              }
              //get NS
              if(param_cnt==2 && param_pos==j && *(data+j+1)!=','){
                _gps_data.lati_ns=*(data+j+1)=='N'?0:1;
                // printf("latitude: %c\n",_gps_data.lati_ns?'S':'N');
              }
              //get Longtitude
              if(param_cnt==3 && param_pos==j && *(data+j+1)!=','){
                _gps_data.long_d=((*(data+j+1)-'0')*100+(*(data+j+2)-'0')*10
                                  +(*(data+j+3)-'0'));
                _gps_data.long_m=((*(data+j+4)-'0')*10+(*(data+j+5)-'0'));
                int m_dec=0;
                for(int k=0; k<5; k++)
                  m_dec=m_dec*10+(*(data+j+7+k)-'0');
                _gps_data.long_m+=m_dec/100000.0;
                // printf("Able to read longtitude: %d %.5f\n",_gps_data.long_d,
                //         _gps_data.long_m);
              }
              //get EW
              if(param_cnt==4 && param_pos==j && *(data+j+1)!=','){
                _gps_data.long_ew=*(data+j+1)=='E'?0:1;
                // printf("longtitude: %c\n",_gps_data.long_ew?'W':'E');
              }
              //get time
              if(param_cnt==5 && param_pos==j && *(data+j+1)!=','){
                _gps_data.time_h=((*(data+j+1)-'0')*10+(*(data+j+2)-'0'));
                _gps_data.time_m=((*(data+j+3)-'0')*10+(*(data+j+4)-'0'));
                _gps_data.time_s=((*(data+j+5)-'0')*10+(*(data+j+6)-'0'));
                int m_dec=0;
                for(int k=0; k<2; k++)
                  m_dec=m_dec*10+(*(data+j+8+k)-'0');
                _gps_data.time_s+=m_dec/100.0;
                printf("Able to read time: %d : %d : %.1f\n",_gps_data.time_h,
                        _gps_data.time_m,_gps_data.time_s);
              }
              break;
            }
            default:{   //exit case
              j=e_pos;
              break;
            }
          }
        }//end of for (parameter)
      }//end of for (frame)
      vTaskDelay(200/portTICK_RATE_MS);
      //if frame_cnt>0 trigger transmission
      if(frame_cnt>0){
        _gps_data_rdy=true;
      }
    }else
      vTaskDelay(100/portTICK_RATE_MS);
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
    espnow_send_param_t send_param;
    memset(&send_param, 0, sizeof(espnow_send_param_t));
    espnow_startup_init(&send_param);

    gps_uart_init();

    //begin espnow loop
    xTaskCreate(espnow_routine_task, "espnow_routine_task", 2048, &send_param, 4, NULL);

    //begin loop for gps
    xTaskCreate(gps_routine_task, "gps_routine_task" , 2048, NULL, 2, NULL);

}
