#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// Q8bot-specific Modules
#include "systemParams.h"

// Initialize global objects
esp_now_peer_info_t peerInfo;

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

void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // Validate minimum length
  if (len < 1) return;

  if (data[0] == PAIRING) {
    // Validate PAIRING message length
    if (len < sizeof(PairingMessage)) {
      Serial.println("Invalid pairing response length");
      return;
    }
    memcpy(&pairingData, data, sizeof(PairingMessage));
    // Serial.println("Paired with server: "); printMAC(mac);
    memcpy(serverMac, mac, sizeof(serverMac));
    addPeer(serverMac);
    paired = true;
  } else if (data[0] == DATA) {
    // Validate DATA message length
    if (len < sizeof(IntMessage)) {
      Serial.println("Invalid data message length");
      return;
    }
    memcpy(&recvMsg, data, sizeof(IntMessage));
    for (int i = 0; i < 100; i++) {
      Serial.print(recvMsg.data[i]);
      Serial.print(" ");
    } Serial.println();
    memset(&recvMsg, 0, sizeof(recvMsg));
  }
}

// Callback when data is sent. Not used ATM
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  // delay(2000);  // Useful for debugging

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.macAddress(clientMac);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(OnDataSent);
  addPeer(broadcastMAC);

  // Prepare pairing request message
  pairingData.id = 1;
  memcpy(pairingData.macAddr, clientMac, sizeof(clientMac));
  pairingData.channel = chan;
  lastPairAttempt = millis();
}

void loop() {
  if (!paired && millis() - lastPairAttempt > 2000) {
    lastPairAttempt = millis();
    // Serial.println("Sending a pairing request...");
    esp_now_send(broadcastMAC, (uint8_t*)&pairingData, sizeof(pairingData));
  } else if (Serial.available()) {
    int bytesRead = Serial.readBytesUntil(';', sendMsg.data, sizeof(sendMsg.data) - 1);  // Read until newline or buffer size limit
    if (bytesRead > 0) {
      sendMsg.data[bytesRead] = '\0';  // Null-terminate the string
    }

    // Send to receiver's mac address
    sendMsg.msgType = DATA;
    sendMsg.id = 1;
    esp_err_t result = esp_now_send(serverMac, (uint8_t*)&sendMsg, sizeof(sendMsg));
  }
  delay(1);
}