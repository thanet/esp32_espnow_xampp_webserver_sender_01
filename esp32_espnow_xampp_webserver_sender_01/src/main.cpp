/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-esp-now-wi-fi-web-server/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "SHT3xSensor.h"

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1

// // Digital pin connected to the DHT sensor
// #define DHTPIN 4  

// // Uncomment the type of sensor in use:
// //#define DHTTYPE    DHT11     // DHT 11
// #define DHTTYPE    DHT22     // DHT 22 (AM2302)
// //#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// DHT dht(DHTPIN, DHTTYPE);

//MAC Address of the receiver 
//uint8_t broadcastAddress[] = {0xCC, 0xDB, 0xA7, 0x31, 0xBF, 0x48};  // for specific reciever
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};    // for boardcast , non specific reciever

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int read_module_no;
    float temp;
    float hum;
    int readingId;
} struct_message;

esp_now_peer_info_t peerInfo;

//Create a struct_message called myData
struct_message myData;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

unsigned int readingId = 0;

// Insert your SSID
constexpr char WIFI_SSID[] = "True Enjoy";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}


// for sensor read data  Globle var
SHT3xSensor sht3x;
// Phototype function
  void Sht30_Reading();

float readDHTTemperature() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = random(25,35);
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  myData.temp = t;
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  else {
    Serial.println(t);
    return t;
  }
}

float readDHTHumidity() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = random(75,99);
  myData.hum = h;
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  else {
    Serial.println(h);
    return h;
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println("inside send package are....");
  Serial.print("read_Module_no = :: "); Serial.println(myData.read_module_no);
  Serial.print("temperature = :: "); Serial.println(myData.temp);
  Serial.print("humidity = :: "); Serial.println(myData.hum);
  Serial.print("readingId = :: "); Serial.println(myData.readingId);

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  //Init Serial Monitor
  Serial.begin(115200);

  // dht.begin();
     while (!Serial) {
        delay(100);
    }
    
    if (!sht3x.begin()) {
        Serial.println("Failed to initialize SHT3x sensor.");
        while (true); // Stop execution if initialization fails
    }



  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    //Set values to send
    Sht30_Reading();    // function to read data via Sht30

    myData.read_module_no = BOARD_ID;
    //myData.temp = readDHTTemperature();
    //myData.hum = readDHTHumidity();
    myData.readingId = readingId++;
     
    //Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
}


void Sht30_Reading() {
  if (sht3x.readMeasurement(myData.temp, myData.hum)) {
        Serial.print("Sht30Temperature: ");
        Serial.print(myData.temp);
        Serial.print(" C\tHumidity: ");
        Serial.print(myData.hum);
        Serial.println(" %");
    } else {
        Serial.print("Error reading measurement: ");
        Serial.println(sht3x.getLastError());
    }
}