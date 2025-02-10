#include <esp_now.h>
#include <WiFi.h>
#include "ESPNOWConfig.h"

/** This is all the data about the peer **/

/** RECEIVER MAC ADDRESS: 60:55:f9:af:83:ec **/
/** REPLACE WITH YOUR RECEIVER MAC Address **/

uint8_t MACAddress[] = {0xFC, 0xE8, 0xC0, 0x75, 0x99, 0xD0};

esp_now_peer_info_t slave;

/** Structure example to receive data **/ 
/** Must match the sender structure **/ 

sw_state ledData;

sensor DHTsensorRecv;

volatile bool dataReceived = false; /** Check data **/

  /** The all important data! **/
  /** callback when data is sent from Master to Slave **/
  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {


    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    
    // esp_err_t result = esp_now_send(slave.peer_addr, (uint8_t *) &ledData, sizeof(ledData));
    // if (result == ESP_OK) {
    //     Serial.println("Sent successfully");
    // } else {
    //     Serial.println("Send failed");
    // }
  }

  /** callback function that will be executed when data is received **/ 
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  /** Copies the sender mac address to a string **/ 
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&DHTsensorRecv, incomingData, sizeof(DHTsensorRecv));
  
  
  Serial.printf("Board ID %u: %u bytes\n", DHTsensorRecv.id, len);
  Serial.printf("t value: %4.2f \n", DHTsensorRecv.temp);
  Serial.printf("h value: %4.2f \n", DHTsensorRecv.hum);
  Serial.printf("readingID value: %d \n", DHTsensorRecv.readingId);
  Serial.println();

  dataReceived = true;
 
}


void EspNow_init() {

  WiFi.mode(WIFI_STA);

    /** Init ESP-NOW **/ 
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
      }

  /** Once ESPNow is successfully Init, we will register for Send CB to **/ 
  /** Get the status of Trasnmitted packet **/  

  esp_now_register_send_cb(OnDataSent);
  memcpy(slave.peer_addr, MACAddress, 6);
  slave.channel = 0;  
  slave.encrypt = false;

   /** Add peer  **/        
   if (esp_now_add_peer(&slave) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
    /** Get recv packer info **/ 
    esp_now_register_recv_cb(OnDataRecv);



}
/** Send data to Slave **/
void dataSend(){

  ledData.switch_states[0] = switch_states[0];
  ledData.switch_states[1] = switch_states[1];
  ledData.switch_states[2] = switch_states[2];
  ledData.switch_states[3] = switch_states[3];
  ledData.switch_states[4] = switch_states[4];

  esp_now_send(MACAddress, (uint8_t *) &ledData, sizeof(ledData));
  
}