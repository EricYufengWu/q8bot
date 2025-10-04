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
#include "macStorage.h"

// Initialize global objects
esp_now_peer_info_t peerInfo;
HardwareSerial          ser(0);
Dynamixel2Arduino       q8dxl(ser, DXL_DIR_PIN);
q8Dynamixel             q8(q8dxl);
bool started = false;  // Track robot start state
macStorage storage;

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
  // Validate minimum length
  if (len < 1) return;

  uint8_t msgType = data[0];
  if (msgType == PAIRING && !paired) {
    // Validate PAIRING message length
    if (len < sizeof(PairingMessage)) return;
    memcpy(&pairingData, data, sizeof(pairingData));
    if (debugMode) {
      Serial.print("[PAIRING] Pairing request from: "); printMAC(mac);
    }
    memcpy(clientMac, mac, 6);
    WiFi.softAPmacAddress(pairingData.macAddr);  // Overwrite with our own MAC
    pairingData.channel = chan;
    pairingData.id = 0;  // Server is ID 0
    addPeer(clientMac);
    esp_now_send(clientMac, (uint8_t*)&pairingData, sizeof(pairingData));
    paired = true;
    lastHeartbeatReceived = millis();

    // Save the controller MAC address to EEPROM
    storage.savePeerMAC(clientMac);
    Serial.println("[STORAGE] Saved controller MAC to EEPROM");

    if (debugMode) {
      Serial.println("[PAIRING] Paired successfully");
    }
  } else if (msgType == HEARTBEAT && paired) {
    // Echo heartbeat back to controller
    if (len < sizeof(HeartbeatMessage)) return;
    lastHeartbeatReceived = millis();
    if (debugMode) {
      Serial.println("[HEARTBEAT] Received, echoing back");
    }
    esp_now_send(mac, data, len);
  } else if (msgType == DATA && paired) {
    // Validate DATA message length
    if (len < sizeof(CharMessage)) return;
    lastHeartbeatReceived = millis();  // Any DATA also counts as "alive"
    memcpy(&theirMsg, data, sizeof(theirMsg));
    uint8_t result = q8.parseData(theirMsg.data);
    // myMsg params
    myMsg.id = 0;  // Server ID
    if (result == 0) {  // No special instruction.
      return;
    } else {            // When controller requests battery legel or data
      switch (result) {
        case 1: {
          if (debugMode) {
            Serial.println("[DATA] Send battery level");
          }
          myMsg.data[0] = (uint16_t)FuelGauge.percent();
          esp_now_send(clientMac, (uint8_t *) &myMsg, sizeof(myMsg));
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
            if (debugMode) {
              Serial.println("[DATA] All recorded data: ");
              for (size_t i = 0; i < masterSize; ++i) {
                Serial.print(rData[i]); Serial.print(" ");
              } Serial.println();
            }
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
                  memset(&myMsg.data[currentChunkSize], 0, (chunkSize - currentChunkSize));
              }
              // Send the chunk via ESP-NOW
              esp_now_send(clientMac, (uint8_t *)&myMsg, sizeof(myMsg));
              // Update offset and Reset myData
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

void unpair() {
  if (debugMode) {
    Serial.println("[HEARTBEAT] Connection lost - returning to pairing mode");
  }
  esp_now_del_peer(clientMac);
  storage.clearPeerMAC();
  Serial.println("[STORAGE] Cleared controller MAC from EEPROM");
  memset(clientMac, 0, sizeof(clientMac));
  paired = false;
  started = false;
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

  // Check for saved MAC address
  if (storage.loadPeerMAC(clientMac)) {
    Serial.print("[STORAGE] Found saved controller MAC: "); printMAC(clientMac);
    addPeer(clientMac);
    paired = true;
    lastHeartbeatReceived = millis();
    Serial.println("[STORAGE] Attempting to reconnect to saved controller");
  } else {
    Serial.println("[STORAGE] No saved MAC found - waiting for pairing request");
  }

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
  static unsigned long lastBlink = 0;
  unsigned long now = millis();

  // Check for debug mode toggle
  if (Serial.available()) {
    char c = Serial.peek();
    if (c == 'd') {
      Serial.read(); // Consume the 'd'
      debugMode = !debugMode;
      Serial.print("Debug mode: ");
      Serial.println(debugMode ? "ON" : "OFF");
      return;
    }
  }

  // Check for heartbeat timeout when paired
  if (paired) {
    // Avoid underflow: only check timeout if now >= lastHeartbeatReceived
    if (now >= lastHeartbeatReceived) {
      unsigned long timeSinceLastMsg = now - lastHeartbeatReceived;
      if (timeSinceLastMsg > HEARTBEAT_TIMEOUT_ROBOT) {
        if (debugMode) {
          Serial.print("[HEARTBEAT] Timeout detected (");
          Serial.print(timeSinceLastMsg);
          Serial.println("ms since last message)");
        }
        q8.toggleTorque(0);        // Disable torque hardware
        q8.resetTorqueState();     // Sync internal flag to match disabled state
        unpair();
      }
    }
  }

  // Not paired
  if (!paired) {
    if (now - lastBlink >= 2000) {
      lastBlink = now;
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      if (debugMode) {
        Serial.println("[PAIRING] Waiting for pairing...");
      }
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
          delay(200);
          digitalWrite(LED_PIN, LOW);
          delay(300);
        }
        if (debugMode) {
          Serial.println("[ROBOT] Waiting for robot start...");
        }
      }
      return;
    }
    started = true;
    if (debugMode) {
      Serial.println("[ROBOT] Robot start!");
    }
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