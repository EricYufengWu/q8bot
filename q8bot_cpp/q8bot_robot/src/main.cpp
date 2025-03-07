#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Dynamixel2Arduino.h>
#include <ESPNowW.h>
#include <MAX17043.h>
#include "q8Dynamixel.h"

// Change this to your controller's MAC address if doing bi-directional ESPNow 24:EC:4A:C9:58:A4
// uint8_t receiver_mac[] = {0xEC, 0xDA, 0x3B, 0x36, 0x10, 0xF0};
uint8_t receiver_mac[] = {0x24, 0xEC, 0x4A, 0xC9, 0x58, 0xA4};

// ESPNow
uint16_t myData[100];
char theirData[100];
bool incoming = false;
esp_now_peer_info_t peerInfo;
// Dynamixel
HardwareSerial ser(0);
Dynamixel2Arduino q8dxl(ser, 8); // DIR pin on Q8bot is 8
q8Dynamixel q8(q8dxl);
bool recordData = false;
size_t masterSize;
size_t smallerSize = 4;
uint16_t* rData = nullptr;
// LED
const uint8_t LED_PIN = D0;
const uint8_t MODE_PIN = D3;
// MAX17043
float raw;
char intStr[10];

// Callback when data is received
void onRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
  //          mac_addr[5]);
  memcpy(&theirData, incomingData, sizeof(theirData));
  // Serial.print("Received from ["); Serial.print(macStr); Serial.print("]: "); 
  // Serial.println(theirData);
  uint8_t result = q8.parseData(theirData);
  if (result == 0) {  // No special instruction.
    return;
  } else {
    switch (result) {
      case 1:
        Serial.println("Send battery level");
        myData[0] = (uint16_t)FuelGauge.percent();
        esp_now_send(receiver_mac, (uint8_t *) &myData, sizeof(myData));
        return;

      case 2: {
        // Serial.print("Record data: ");
        uint16_t* posArray = q8.syncRead();

        // If rData is not nullptr, continue appending posArray to it
        if (rData != nullptr) {
          rData = (uint16_t*)realloc(rData, (masterSize + smallerSize) * sizeof(uint16_t));
          for (size_t j = 0; j < smallerSize; ++j) {
            rData[masterSize + j] = posArray[j];
          }
          masterSize += smallerSize;
        } else {
          // If rData is nullptr, initialize it for the first time
          rData = new uint16_t[smallerSize];
          for (size_t j = 0; j < smallerSize; ++j) {
            rData[j] = posArray[j];
          }
          masterSize = smallerSize;
        }

        // for (size_t j = 0; j < smallerSize; ++j) {
        //     Serial.print(posArray[j]); Serial.print(" ");
        // } Serial.println();
        delete[] posArray;
        return; }

      case 3:
        Serial.println("All recorded data: ");
        if (rData != nullptr) {
          for (size_t i = 0; i < masterSize; ++i) {
            Serial.print(rData[i]);
            Serial.print(" ");
          } Serial.println();
          size_t totalSize = masterSize;  // total data in rData
          size_t chunkSize = 100;  // Chunk size for each ESPNow send
          size_t offset = 0;  // To track the position in rData

          // Loop to send the data in chunks
          while (offset < totalSize) {
            // If rData is less than chunk size then use its size instead.
            size_t currentChunkSize = (totalSize - offset < chunkSize) ? (totalSize - offset) : chunkSize;
            // Copy the chunk from rData to myData
            memcpy(myData, &rData[offset], currentChunkSize * sizeof(uint16_t));
            // If the chunk is smaller than 100, fill the remaining spaces with zeros
            if (currentChunkSize < chunkSize) {
                memset(&myData[currentChunkSize], 0, (chunkSize - currentChunkSize));  // Fill remaining with zeros
            }
            // Send the chunk via ESP-NOW
            esp_now_send(receiver_mac, (uint8_t *)myData, currentChunkSize * sizeof(uint16_t));
            // Updata offset now that the current chunk is sent
            offset += currentChunkSize;
            // Reset myData in preparation for next chunk
            memset(myData, 0, sizeof(myData));
          }

          // Reset rData to nullptr
          delete[] rData;
          rData = nullptr;
          masterSize = 0;  // Reset masterSize as well
        } else {
          Serial.println("No data to send");
        }
        return;
    }
  }
}

// Callback when data is sent. Not used ATM
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void displayReading()
{
  // MAX17043 Demo code. To be cleaned up
  Serial.println("Device Reading:");
  // Serial.print("Address:       0x"); Serial.println(FuelGauge.address(), HEX);
  // Serial.print("Version:       "); Serial.println(FuelGauge.version());
  // Serial.print("ADC:           "); Serial.println(FuelGauge.adc());
  Serial.print("Voltage:       "); Serial.print(FuelGauge.voltage()); Serial.println(" mV");
  Serial.print("Percent:       "); Serial.print(FuelGauge.percent()); Serial.println("%");
  // Serial.print("Is Sleeping:   "); Serial.println(FuelGauge.isSleeping() ? "Yes" : "No");
  // Serial.print("Alert:         "); Serial.println(FuelGauge.alertIsActive() ? "Yes" : "No");
  // Serial.print("Threshold:     "); Serial.print(FuelGauge.getThreshold()); Serial.println("%");
  // Serial.print("Compensation:  0x"); Serial.println(FuelGauge.compensation(), HEX);
  Serial.println();
}

void addElementToArray(uint16_t*& array, size_t& currentSize, uint16_t newElement) {
    // Allocate a new array with one extra element
    uint16_t* newArray = new uint16_t[currentSize + 1];
    // Add new element to front
    newArray[0] = newElement;
    // Copy the existing elements to the new array
    memcpy(newArray + 1, array, currentSize * sizeof(uint16_t));
    // Delete the old array (free the memory)
    delete[] array;
    // Update the original pointer and size
    array = newArray;
    currentSize++;  // Increment the size of the array
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MODE_PIN, OUTPUT);

  // // Only needed to grab the MAC address. Disable and upload again.
  // while(!Serial){
  //   delay(100);
  // }
  // Serial.println("Serial port initialized.\n");
  // delay(5000);

  Serial.println("q8bot ESPNOW receiver:");
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();

  // ESPNow Init
  ESPNow.init();
  ESPNow.reg_recv_cb(onRecv);
  // Register for Send CB. Register pair.
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, receiver_mac, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;      
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // MAX17043
  // Initialize the fuel gauge.
  if (FuelGauge.begin()){
    // Reset the device.
    FuelGauge.reset();
    delay(250);
    // Issue a quickstart command and wait for the device to be ready.
    FuelGauge.quickstart();
    delay(125);
    // Display an initial reading.
    displayReading();
  } else{
    Serial.println("MAX17043 NOT found.\n");
  }

  q8.begin();
  digitalWrite(MODE_PIN, HIGH);
}

void loop() {
  while (!q8.commStart()){
    delay(1000);
    Serial.println("Waiting");
  }
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Robot start!");

  while(1){
    delay(9800);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    // q8.setProfile(0);
  }
}

/*
New main loop logic
Have an extra parameter to process. Contains the type of info the controller is requesting. Examples include: 
  - Battery level
  - Current reading from the past 5 seconds of the gait
  - IMU reading from the past 5 seconds of the gait (only roll and pitch is ok)
When receiving this command, the robot starts the movement while simultaneous records
data, but hold on to them until the current movement is over. Then after the current 
movement has terminated, it will send the info in bulk to the controller which passes 
all data to the python script. The python script will do all post processing.

If movement duration was less than the duration of data requested, the robot will 
return as much data as there was. Otherwise, it will only send back the most recent 
5-second of data, or something similar depending on what the controller asks.
*/
