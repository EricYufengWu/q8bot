#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>

// Q8bot-specific Modules
#include "systemParams.h"
#include "macStorage.h"

// Initialize global objects
esp_now_peer_info_t peerInfo;
Preferences prefs;
macStorage storage;

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
    if (len < sizeof(PairingMessage)) return;
    memcpy(&pairingData, data, sizeof(PairingMessage));
    if (debugMode) {
      Serial.print("[PAIRING] Paired with server: "); printMAC(mac);
    }
    memcpy(serverMac, mac, sizeof(serverMac));
    addPeer(serverMac);
    paired = true;
    lastHeartbeatReceived = millis();

    // Save the MAC address to EEPROM
    storage.savePeerMAC(serverMac);
    Serial.println("[STORAGE] Saved peer MAC to EEPROM");

    if (debugMode) {
      Serial.println("[HEARTBEAT] Connection established, heartbeat timer started");
    }
  } else if (data[0] == HEARTBEAT) {
    // Robot echoed heartbeat back
    if (len < sizeof(HeartbeatMessage)) return;
    lastHeartbeatReceived = millis();
    if (debugMode) {
      HeartbeatMessage hbMsg;
      memcpy(&hbMsg, data, sizeof(HeartbeatMessage));
      uint32_t rtt = millis() - hbMsg.timestamp;
      Serial.print("[HEARTBEAT] ACK received, RTT: ");
      Serial.print(rtt);
      Serial.println("ms");
    }
  } else if (data[0] == DATA) {
    // Validate DATA message length
    if (len < sizeof(IntMessage)) return;
    memcpy(&recvMsg, data, sizeof(IntMessage));
    lastHeartbeatReceived = millis();  // Any DATA also counts as "alive"
    for (int i = 0; i < 100; i++) {
      Serial.print(recvMsg.data[i]);
      Serial.print(" ");
    } Serial.println();
    memset(&recvMsg, 0, sizeof(recvMsg));
  }
}

void unpair() {
  if (debugMode) {
    Serial.println("[HEARTBEAT] Connection lost - returning to pairing mode");
  }
  esp_now_del_peer(serverMac);
  storage.clearPeerMAC();
  Serial.println("[STORAGE] Cleared peer MAC from EEPROM");
  memset(serverMac, 0, sizeof(serverMac));
  paired = false;
  lastPairAttempt = millis();
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

  // Check for saved MAC address
  if (storage.loadPeerMAC(serverMac)) {
    Serial.print("[STORAGE] Found saved MAC: "); printMAC(serverMac);
    addPeer(serverMac);
    paired = true;
    lastHeartbeatReceived = millis();
    lastHeartbeatSent = millis();
    Serial.println("[STORAGE] Attempting to reconnect to saved peer");
  } else {
    Serial.println("[STORAGE] No saved MAC found - entering pairing mode");
  }

  // Prepare pairing request message
  pairingData.id = 1;
  memcpy(pairingData.macAddr, clientMac, sizeof(clientMac));
  pairingData.channel = chan;
  lastPairAttempt = millis();
}

void loop() {
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

  if (!paired) {
    // Pairing mode
    if (millis() - lastPairAttempt > 2000) {
      lastPairAttempt = millis();
      if (debugMode) {
        Serial.println("[PAIRING] Sending pairing request...");
      }
      esp_now_send(broadcastMAC, (uint8_t*)&pairingData, sizeof(pairingData));
    }
  } else {
    // Connected mode

    // Check for connection timeout
    unsigned long timeSinceLastHB = millis() - lastHeartbeatReceived;
    if (timeSinceLastHB > HEARTBEAT_TIMEOUT) {
      if (debugMode) {
        Serial.print("[HEARTBEAT] Timeout detected (");
        Serial.print(timeSinceLastHB);
        Serial.println("ms since last response)");
      }
      unpair();
      return;
    }

    // Send periodic heartbeat
    if (millis() - lastHeartbeatSent > HEARTBEAT_INTERVAL) {
      lastHeartbeatSent = millis();
      heartbeatMsg.msgType = HEARTBEAT;
      heartbeatMsg.id = 1;
      heartbeatMsg.timestamp = millis();
      if (debugMode) {
        Serial.print("[HEARTBEAT] Sending heartbeat (last response: ");
        Serial.print(timeSinceLastHB);
        Serial.println("ms ago)");
      }
      esp_now_send(serverMac, (uint8_t*)&heartbeatMsg, sizeof(heartbeatMsg));
    }

    // Handle serial data
    if (Serial.available()) {
      int bytesRead = Serial.readBytesUntil(';', sendMsg.data, sizeof(sendMsg.data) - 1);
      if (bytesRead > 0) {
        sendMsg.data[bytesRead] = '\0';
      }

      // Send to receiver's mac address
      sendMsg.msgType = DATA;
      sendMsg.id = 1;
      esp_err_t result = esp_now_send(serverMac, (uint8_t*)&sendMsg, sizeof(sendMsg));
    }
  }
  delay(1);
}