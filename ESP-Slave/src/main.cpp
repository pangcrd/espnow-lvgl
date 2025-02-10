#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define NUM_LED  4   
int led_pins[NUM_LED] = {5,22,21,19};  /** LED OR RELAY GPIO **/ 

/** Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc) **/ 
#define BOARD_ID 1
//#define BOARD_ID 2
/** MAC Address of the receiver => Master**/
uint8_t MasterAddress[] = {0xD4, 0x8A, 0xFC, 0xC8, 0xE7, 0x64};

/** Digital pin connected to the DHT sensor **/ 
#define DHTPIN 23 

/** Type of sensor in use: **/
#define DHTTYPE    DHT22
DHT dht(DHTPIN, DHTTYPE);   

typedef struct sw_state {
  bool switch_states[5];  /** Create switch state **/ 
} sw_state;

sw_state ledData;

/** Create Struct for Sensor **/
typedef struct sensor {
    int id;
    float temp;
    float hum;
    int readingId;
} sensor;

sensor DHTsensor;

unsigned long previousMillis = 0;   /** Stores last time temperature was published **/ 
const long interval = 2000;        /** Interval at which to publish sensor readings **/ 

unsigned int readingId = 0;

/** Read temperature and humidity **/
void readTempHumi(sensor &DHTdata) { /** Transmit the sensor &DHTdata by reference (&), which means DHTdata is essentially the same as DHTsensor.
  If there are any changes to DHTdata, DHTsensor will change accordingly.**/
  DHTdata.temp = dht.readTemperature();
  DHTdata.hum = dht.readHumidity();

  if (isnan(DHTdata.temp) || isnan(DHTdata.hum)) {
      Serial.println("Failed to read from DHT sensor!");
      DHTdata.temp = 0;
      DHTdata.hum = 0;
  } else {
      Serial.println("Temp: " + String(DHTdata.temp) + "°C, Hum: " + String(DHTdata.hum) + "%");
  }
}

/** callback when data is recv from Master **/
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {

    memcpy(&ledData, incomingData, sizeof(ledData));

    // Serial.println("Data received!");
    // Serial.print("Switch states: ");
    // Serial.print(ledData.switch_states[0]);
    // Serial.print(", ");
    // Serial.print(ledData.switch_states[1]);
    // Serial.print(", ");
    // Serial.println(ledData.switch_states[2]);

    digitalWrite(led_pins[0], ledData.switch_states[1] ? HIGH : LOW);  // LED 1
    digitalWrite(led_pins[1], ledData.switch_states[2] ? HIGH : LOW);  // LED 2
    digitalWrite(led_pins[2], ledData.switch_states[3] ? HIGH : LOW);  // LED 3
    digitalWrite(led_pins[3], ledData.switch_states[4] ? HIGH : LOW);  // LED 4
}
/** Callback when data is sent to master **/ 
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  

}

void setup() {

  Serial.begin(115200);
  dht.begin();
  
  /** Set OUTPUT pin **/
  for (int i = 0; i < NUM_LED; i++) pinMode(led_pins[i], OUTPUT);

  WiFi.mode(WIFI_STA);
  //WiFi.softAP("RX_1", "RX_1_Password", CHANNEL, 0);
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  /** Once ESPNow is successfully Init, we will register for Send CB to **/ 
  /** Get the status of Trasnmitted packet **/ 
  esp_now_register_send_cb(OnDataSent);
  
  /** Register peer master **/
  esp_now_peer_info_t master;
  memset(&master, 0, sizeof(master));
  memcpy(master.peer_addr, MasterAddress, 6); /** Master and slave must be run on the same channel. I choose channel 6 **/
  master.channel = 0; 
  master.encrypt = false;
  
  /** Add peer  **/       
  if (esp_now_add_peer(&master) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_init();
  
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    /**  Save the last time a new reading was published **/
    previousMillis = currentMillis;
    /** Set values to send **/
    DHTsensor.id = BOARD_ID;
    readTempHumi(DHTsensor);
    DHTsensor.readingId = readingId++;
     
    /** Send message via ESP-NOW **/
    esp_err_t result = esp_now_send(MasterAddress, (uint8_t *) &DHTsensor, sizeof(DHTsensor));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
}

