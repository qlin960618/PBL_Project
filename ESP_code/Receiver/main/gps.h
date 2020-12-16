#ifndef GPS_H
#define GPS_H


#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp32/rom/crc.h"

#define UART_RTS  (UART_PIN_NO_CHANGE)
#define UART_CTS  (UART_PIN_NO_CHANGE)
//UART Parameter
#define BUF_SIZE (1024)


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
} __attribute__((packed)) gps_data_t;



extern gps_data_t _gps_data;
extern bool new_data;

extern void gps_init(uart_port_t uart_port, gpio_num_t gps_rx_pin,
                     gpio_num_t gps_tx_pin);

extern void gps_watcher_init(uint32_t depth, UBaseType_t priority);

extern void gps_data_prepare(uint8_t *m_payload, uint16_t *m_crc);
extern void gps_data_parser(gps_data_t *gps_data, uint8_t *m_payload,
                    uint16_t m_crc);

extern bool gps_data_rdy(void);

extern bool gps_get_data(gps_data_t * data);




























#endif
