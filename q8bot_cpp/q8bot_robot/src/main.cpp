#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Dynamixel2Arduino.h>
#include <MAX17043.h>

// Q8bot-specific Modules
#include "q8Dynamixel.h"
#include "userParams.h"
#include "systemParams.h"
#include "pinMapping.h"

// Initialize global objects
esp_now_peer_info_t peerInfo;
HardwareSerial          ser(0);
Dynamixel2Arduino       q8dxl(ser, DXL_DIR_PIN);
q8Dynamixel             q8(q8dxl);

// Helper functions for ESP-NOW
void printMAC(const uint8_t* mac) {
  char buff[18];
  sprintf(buff, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(buff);
}

bool addPeer(const uint8_t* mac) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = chan;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
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
    newArray[0] = newElement;
    memcpy(newArray + 1, array, currentSize * sizeof(uint16_t));
    // Delete the old array (free the memory)
    delete[] array;
    // Update the original pointer and size
    array = newArray;
    currentSize++;
}

// Callbacks
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  uint8_t msgType = data[0];
  if (msgType == PAIRING && !paired) {
    memcpy(&pairingData, data, sizeof(pairingData));
    Serial.print("Pairing request from: "); printMAC(mac);
    memcpy(clientMac, mac, 6);
    WiFi.softAPmacAddress(pairingData.macAddr);  // Overwrite with our own MAC
    pairingData.channel = chan;
    pairingData.id = 0;  // Server is ID 0
    addPeer(clientMac);
    esp_now_send(clientMac, (uint8_t*)&pairingData, sizeof(pairingData));
    paired = true;
  } else if (msgType == DATA) {
    memcpy(&theirMsg, data, sizeof(theirMsg));
    uint8_t result = q8.parseData(theirMsg.data);
    // myMsg params
    myMsg.id = 0;  // Server ID
    if (result == 0) {  // No special instruction.
      return;
    } else {            // When controller requests battery legel or data
      switch (result) {
        case 1: {
          Serial.println("Send battery level");
          myMsg.data[0] = (uint16_t)FuelGauge.percent();
          esp_now_send(receiver_mac, (uint8_t *) &myMsg, sizeof(myMsg));
          return;
        }

        case 2: {
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
          delete[] posArray;
          return; 
        }

        case 3: {
          if (rData != nullptr) {
            // Serial.println("All recorded data: ");
            // for (size_t i = 0; i < masterSize; ++i) {
            //   Serial.print(rData[i]); Serial.print(" ");
            // } Serial.println();
            size_t totalSize = masterSize;  // total data in rData
            size_t chunkSize = 100;         // Chunk size for each ESPNow send
            size_t offset = 0;              // To track the position in rData

            // Loop to send the data in chunks
            while (offset < totalSize) {
              // If rData is less than chunk size then use its size instead.
              size_t currentChunkSize = (totalSize - offset < chunkSize) ? (totalSize - offset) : chunkSize;
              memcpy(myMsg.data, &rData[offset], currentChunkSize * sizeof(uint16_t));
              // If the chunk is smaller than 100, fill the remaining spaces with zeros
              if (currentChunkSize < chunkSize) {
                  memset(&myMsg.data[currentChunkSize], 0, (chunkSize - currentChunkSize));  // Fill remaining with zeros
              }
              // Send the chunk via ESP-NOW
              esp_now_send(receiver_mac, (uint8_t *)&myMsg, currentChunkSize * sizeof(uint16_t));
              // Updata offset and Reset myData
              offset += currentChunkSize;
              memset(myMsg.data, 0, sizeof(myMsg.data));
            }
            delete[] rData;
            rData = nullptr;
            masterSize = 0;
          }
          return;
        }
      }
    }
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MODE_PIN, OUTPUT);
  // delay(2000);  // Useful for debugging

  // Init Wi-Fi
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("esp-server", nullptr, chan); // Optional, just to enable softAP mode
  chan = WiFi.channel();
  WiFi.softAPmacAddress(serverMac);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);  // Set up callback when data is received.

  // Serial.println("q8bot ESPNOW receiver:");
  // WiFi.mode(WIFI_MODE_STA);
  // Serial.println(WiFi.macAddress());
  // WiFi.disconnect();

  // // ESPNow Init
  // ESPNow.init();
  // ESPNow.reg_recv_cb(onRecv);
  // // Register for Send CB. Register pair.
  // esp_now_register_send_cb(OnDataSent);
  // memcpy(peerInfo.peer_addr, receiver_mac, 6);
  // peerInfo.channel = 0;  
  // peerInfo.encrypt = false;      
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add peer");
  //   return;
  // }

  // MAX17043 Init
  if (FuelGauge.begin()){
    FuelGauge.reset(); // Reset the device.
    delay(250);
    FuelGauge.quickstart();
    delay(125);
    displayReading(); // Display an initial reading.
  } else{
    Serial.println("MAX17043 NOT found.\n");
  }

  q8.begin();
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);
  delay(200);
}

void loop() {
  static bool started = false;
  static unsigned long lastBlink = 0;
  unsigned long now = millis();

  // Not paired
  if (!paired) {
    if (now - lastBlink >= 2000) {
      lastBlink = now;
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
      digitalWrite(LED_PIN, LOW);
      Serial.println("Waiting for pairing...");
    }
    return;
  }

  // Paired but not started
  if (!started) {
    if (!q8.commStart()) {
      if (now - lastBlink >= 2000) {
        lastBlink = now;
        for (int i = 0; i < 2; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(1000);
          digitalWrite(LED_PIN, LOW);
          delay(1000);
        }
        Serial.println("Waiting for robot start...");
      }
      return;
    }
    started = true;
    Serial.println("Robot start!");
  }

  // Robot started
  if (now - lastBlink >= 10000) {
    lastBlink = now;
    for (int brightness = 0; brightness <= 255; brightness++) {
      analogWrite(LED_PIN, brightness);
      delay(5); // Adjust for speed
    }
    for (int brightness = 255; brightness >= 0; brightness--) {
      analogWrite(LED_PIN, brightness);
      delay(5); // Adjust for speed
    }
  }
}