/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef SYSTEM_TRANSMITTER_H
#define SYSTEM_TRANSMITTER_H


/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

// #include "gps.h"


//GPS Data Type
typedef struct {
    uint8_t fix_status;   //current fix status of the GPS
    uint8_t lati_d;       //Latitude in Degree
    double  lati_m;       //Latitude in minutes
    uint8_t lati_ns;      //N or S == 0 or 1
    uint8_t long_d;       //Longtitude in Degree
    double  long_m;       //Longtitude in Minutes
    uint8_t long_ew;      //E or W == 0 or 1
    uint8_t time_h;       //UTC time H
    uint8_t time_m;       //UTC time M
    float   time_s;        //UTC time S
} __attribute__((packed)) esp_gps_data_t;


/* Parameters of sending ESPNOW data. */
typedef struct {
    bool unicast;                         //Send unicast ESPNOW data.
    bool broadcast;                       //Send broadcast ESPNOW data.
    uint8_t state;                        //Indicate that if has received broadcast ESPNOW data or not.
    uint32_t magic;                       //Magic number which is used to determine which device to send unicast ESPNOW data.
    uint16_t count;                       //Total count of unicast ESPNOW data to be sent.
    int len;                              //Length of ESPNOW data to be sent, unit: byte.
    // uint8_t buffer[sizeof(esp_gps_data_t)+8];//Buffer pointing to ESPNOW data.
    uint8_t buffer[sizeof(esp_gps_data_t)+8];//Buffer pointing to ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   //MAC address of destination device.
} espnow_send_param_t;

#endif
