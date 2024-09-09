/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-esp-now-wi-fi-web-server/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
*/

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "SHT3xSensor.h"

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1

// MAC Address of the receiver 
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // For broadcast, non-specific receiver

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int read_module_no;
    float temp;
    float hum;
    int readingId;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

unsigned int readingId = 0;

// Define multiple SSIDs
constexpr const char* WIFI_SSIDS[] = {"True Enjoy", "ENJMesh"};  // `const char*` to prevent modification
constexpr size_t SSID_COUNT = sizeof(WIFI_SSIDS) / sizeof(WIFI_SSIDS[0]);

// Sensor object
SHT3xSensor sht3x;
bool sht3xAvailable = false;  // Track sensor availability

// Function prototypes
void Sht30_Reading();
void initializeWiFi();
void initializeEspNow();
float readDHTTemperature();
float readDHTHumidity();
int32_t getWiFiChannel(const char *ssid);
const char* connectToAvailableSSID();

// Wi-Fi channel function
int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println("Inside send package are....");
  Serial.print("read_Module_no = :: "); Serial.println(myData.read_module_no);
  Serial.print("temperature = :: "); Serial.println(myData.temp);
  Serial.print("humidity = :: "); Serial.println(myData.hum);
  Serial.print("readingId = :: "); Serial.println(myData.readingId);

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  while (!Serial) {
    delay(100);
  }

  // Attempt to initialize the SHT30 sensor
  sht3xAvailable = sht3x.begin();

  if (sht3xAvailable) {
    Serial.println("SHT3x sensor initialized successfully.");
  } else {
    Serial.println("SHT3x sensor not found, using random data instead.");
  }

  // Initialize Wi-Fi and ESP-NOW
  initializeWiFi();
  initializeEspNow();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;

    // Set values to send
// ++for test with out SHT30
    if (!sht3xAvailable) {  
      Sht30_Reading();  // Function to read data via SHT30
    } else {
      // Generate random data since SHT30 is not available
      myData.temp = readDHTTemperature();
      myData.hum = readDHTHumidity();
    }
// --for test with out SHT30
// ++for test with SHT30
    // if (sht3xAvailable) {  
    //   Sht30_Reading();  // Function to read data via SHT30
    // } else {
    //   // Generate random data since SHT30 is not available
      
    // }
// --for test with SHT30

    myData.read_module_no = BOARD_ID;
    myData.readingId = readingId++;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
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

// Function to read temperature randomly (for simulation)
float readDHTTemperature() {
  float t = random(25, 35);  // Generate a random temperature value
  if (isnan(t)) {
    Serial.println("Failed to read simulated temperature!");
    return 0;
  } else {
    Serial.println(t);
    return t;
  }
}

// Function to read humidity randomly (for simulation)
float readDHTHumidity() {
  float h = random(75, 99);  // Generate a random humidity value
  if (isnan(h)) {
    Serial.println("Failed to read simulated humidity!");
    return 0;
  } else {
    Serial.println(h);
    return h;
  }
}

// Initialize Wi-Fi and set the appropriate channel
void initializeWiFi() {
  WiFi.mode(WIFI_STA);
  const char* connectedSSID = connectToAvailableSSID();

  if (connectedSSID != nullptr) {
    int32_t channel = getWiFiChannel(connectedSSID);
    WiFi.printDiag(Serial);  // Uncomment to verify channel number before
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    WiFi.printDiag(Serial);  // Uncomment to verify channel change after
  }
}

// Initialize ESP-NOW
void initializeEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

// Connect to the first available SSID from the list
const char* connectToAvailableSSID() {
  for (size_t i = 0; i < SSID_COUNT; i++) {
    const char* ssid = WIFI_SSIDS[i];
    int32_t channel = getWiFiChannel(ssid);
    if (channel > 0) {
      Serial.print("Connecting to SSID: ");
      Serial.println(ssid);
      WiFi.begin(ssid);
      if (WiFi.waitForConnectResult() == WL_CONNECTED) {
        Serial.print("Connected to SSID: ");
        Serial.println(ssid);
        return ssid;
      }
    }
  }
  Serial.println("Failed to connect to any SSID.");
  return nullptr;
}
