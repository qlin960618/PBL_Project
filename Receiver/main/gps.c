#include "gps.h"

// #define VERBOSE_M

static uart_port_t _uart_port;
static gpio_num_t _gps_rx_pin;
static gpio_num_t _gps_tx_pin;

gps_data_t _gps_data;
bool _new_data=false;

void gps_init(uart_port_t uart_port, gpio_num_t gps_rx_pin,
                     gpio_num_t gps_tx_pin)
{
  _uart_port=uart_port;
  _gps_tx_pin=gps_tx_pin;
  _gps_rx_pin=gps_rx_pin;

  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(_uart_port, &uart_config);
  uart_set_pin(_uart_port, _gps_tx_pin, _gps_rx_pin, UART_RTS, UART_CTS);
  esp_err_t err = uart_driver_install(_uart_port, BUF_SIZE * 2, 0, 0, NULL, 0);
  printf("%s\n",esp_err_to_name(err));
  printf("uart Initialize complete\n");

}


static void gps_routine_task(){

  uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
  uart_flush(_uart_port);
  while(1)
  {
    int len = uart_read_bytes(_uart_port, data, BUF_SIZE,
                              200 / portTICK_RATE_MS);
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
                  #ifdef VERBOSE_M
                    printf("Able to read fix val: %d\n",_gps_data.fix_status);
                  #endif
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
                #ifdef VERBOSE_M
                  printf("Able to read time: %d : %d : %.1f\n",_gps_data.time_h,
                          _gps_data.time_m,_gps_data.time_s);
                #endif
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
      vTaskDelay(100/portTICK_RATE_MS);
      //if frame_cnt>0 trigger transmission
      if(frame_cnt>0){
        _new_data=true;
      }
    }
      // vTaskDelay(100/portTICK_RATE_MS);

  }
}

void gps_watcher_init(uint32_t depth, UBaseType_t priority){
  printf("Begin watching GPS data\n");
  //begin xTask
  xTaskCreate(gps_routine_task, "gps_routine_task" , depth, NULL, priority, NULL);

}

//prepare GPS data
void gps_data_prepare(uint8_t *m_payload, uint16_t *m_crc)
{
  #ifdef VERBOSE_M
    printf("Parsing GPS data to array\n");
  #endif

  memset(m_payload, 0, sizeof(gps_data_t));
  memcpy(m_payload+21, &(_gps_data.time_h), 6);
  if(_gps_data.fix_status != 0)
  {
    memcpy(m_payload, &_gps_data, 21);
    //crc checksum
    *m_crc = crc16_le(UINT16_MAX, (uint8_t const *)m_payload, sizeof(gps_data_t));
  }else ////no fix
  {
    *m_crc = crc16_le(UINT16_MAX, (uint8_t const *)m_payload, sizeof(gps_data_t));
  }
  _new_data=false;
}

void gps_data_parser(gps_data_t *gps_data_f, uint8_t *m_payload,
                    uint16_t m_crc)
{
  if(m_crc != crc16_le(UINT16_MAX, (uint8_t const *)m_payload,
                        sizeof(gps_data_t)))
  {
    printf("CRC missmatched\n");
    gps_data_f->fix_status=0;
  }else if( *(m_payload)==0 )
  {
    printf("Reciving GPS has no fix\n");
    gps_data_f->fix_status=0;
    memcpy(&(gps_data_f->time_h), m_payload+21, 6);
  }
  else
  {
    memcpy(gps_data_f, m_payload, sizeof(gps_data_t));
  }
}


bool gps_data_rdy(void)
{
  return _new_data;
}

bool gps_get_data(gps_data_t * data){
  memcpy(data, &_gps_data, sizeof(gps_data_t));

  if(_new_data){
    _new_data=false;
    return true;
  }
  return false;
}
