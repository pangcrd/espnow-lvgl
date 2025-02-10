#ifndef ESPNOWCONFIG_H
#define ESPNOWCONFIG_H

#include <esp_now.h>
#include <WiFi.h>

extern bool switch_states[];

/** Struct send state LED**/
typedef struct sw_state{

  bool switch_states[5];  /** Test 5 LED **/ 

} sw_state;

/** Struct recv sensor data**/
typedef struct sensor {
    int id;
    float temp;
    float hum;
    int readingId;
  } sensor;

extern sensor DHTsensorRecv;

extern volatile bool dataReceived ;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len);
void EspNow_init();
void dataSend();

#endif
