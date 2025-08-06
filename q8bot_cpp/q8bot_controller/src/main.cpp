#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "systemParams.h"

// This Q8bot's MAC address. Change this to yours.
// uint8_t receiver_mac[] = {0x24, 0xEC, 0x4A, 0xC9, 0x5D, 0x20};
// uint8_t receiver_mac[] = {0x54, 0x32, 0x04, 0x86, 0xF8, 0xC8};

// // Char array for sending and receiving data
// char myData[100];
// // char theirData[100];
// uint16_t theirData[100];
// bool incoming = false;

// // String for storing serial command
// String cmd;

// PeerInfo
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
  if (data[0] == PAIRING) {
    memcpy(&pairingData, data, sizeof(PairingMessage));
    Serial.println("Paired with server: "); printMAC(mac);
    memcpy(serverMac, mac, sizeof(serverMac));
    addPeer(serverMac);
    paired = true;
  } else if (data[0] == DATA) {
    for (int i = 0; i < 100; i++) {
      Serial.print(dataMsg.data[i]);
      Serial.print(" ");
    } Serial.println();
    memset(&dataMsg, 0, sizeof(dataMsg));
  }
}

// // Callback when data is received
// void onRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
//   char macStr[18];
//   snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
//            mac_addr[5]);
//   memcpy(&theirData, incomingData, len);
//   // Serial.print("Received from ["); Serial.print(macStr); Serial.print("]: "); 
//   // Serial.println(sizeof(theirData));
//   for (int i = 0; i < 100; i++) {
//     Serial.print(theirData[i]);
//     Serial.print(" ");
//   } Serial.println();
//   memset(theirData, 0, sizeof(theirData));
// }

// Callback when data is sent. Not used ATM
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  delay(2000);

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
  pairingData.msgType = PAIRING;
  pairingData.id = 1;
  memcpy(pairingData.macAddr, clientMac, sizeof(clientMac));
  pairingData.channel = chan;
  lastPairAttempt = millis();
}

void loop() {
  if (!paired && millis() - lastPairAttempt > 2000) {
    lastPairAttempt = millis();
    Serial.println("Sending a pairing request...");
    esp_now_send(broadcastMAC, (uint8_t*)&pairingData, sizeof(pairingData));
  } else if (Serial.available()) {
    int bytesRead = Serial.readBytesUntil(';', dataMsg.data, sizeof(dataMsg.data) - 1);  // Read until newline or buffer size limit
    if (bytesRead > 0) {
      dataMsg.data[bytesRead] = '\0';  // Null-terminate the string
      // Serial.print("Received input from serial monitor: ");
      // Serial.println(dataMsg.myData); 
    }

    // Send to receiver's mac address
    dataMsg.msgType = DATA;
    dataMsg.id = 1;
    esp_err_t result = esp_now_send(serverMac, (uint8_t*)&dataMsg, sizeof(dataMsg));
  }
  delay(1);
}