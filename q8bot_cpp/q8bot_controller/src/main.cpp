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

// ============================================================================
// FreeRTOS Handles (Added for FreeRTOS migration - not yet used)
// ============================================================================
QueueHandle_t rxQueue = NULL;
QueueHandle_t debugQueue = NULL;
QueueHandle_t dataOutputQueue = NULL;
EventGroupHandle_t eventGroup = NULL;

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

    // Signal pairing task to stop broadcasting (if FreeRTOS is running)
    if (eventGroup != NULL) {
      xEventGroupSetBits(eventGroup, EVENT_PAIRED);
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

  // Signal pairing task to resume broadcasting (if FreeRTOS is running)
  if (eventGroup != NULL) {
    xEventGroupSetBits(eventGroup, EVENT_UNPAIRED);
  }
}

// Callback when data is sent. Not used ATM
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// ============================================================================
// FreeRTOS Task: Serial Output / Debug (Priority 1 - LOW)
// ============================================================================
void serialOutputTask(void *param) {
  SerialMessage msg;

  while (1) {
    // Check for serial output messages (blocking with timeout)
    if (xQueueReceive(debugQueue, &msg, pdMS_TO_TICKS(10)) == pdTRUE) {
      // Print based on message type
      if (msg.type == MSG_INFO) {
        // INFO messages always printed
        Serial.print(msg.text);
      } else if (msg.type == MSG_DEBUG && debugMode) {
        // DEBUG messages only when debugMode is ON
        Serial.print(msg.text);
      }
    }

    // Check for debug toggle command
    if (Serial.available()) {
      char c = Serial.peek();
      if (c == 'd') {
        Serial.read(); // Consume the 'd'
        debugMode = !debugMode;
        Serial.printf("Debug mode: %s\n", debugMode ? "ON" : "OFF");
      }
    }

    // Low priority - yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================================
// FreeRTOS Task: Pairing Manager (Priority 0 - LOWEST)
// ============================================================================
void pairingTask(void *param) {
  TickType_t lastWake = xTaskGetTickCount();
  SerialMessage msg;

  while (1) {
    // Check pairing state (use old global for now, will migrate to atomic later)
    bool isPaired = paired;

    if (!isPaired) {
      // Send pairing broadcast every 2s
      if (debugQueue != NULL) {
        msg.type = MSG_DEBUG;
        snprintf(msg.text, sizeof(msg.text), "[PAIRING] Sending broadcast...\n");
        xQueueSend(debugQueue, &msg, 0);
      }

      PairingMessage pairingMsg;
      pairingMsg.msgType = PAIRING;
      pairingMsg.id = 1;
      memcpy(pairingMsg.macAddr, clientMac, 6);
      pairingMsg.channel = chan;

      esp_now_send(broadcastMAC, (uint8_t*)&pairingMsg, sizeof(pairingMsg));

      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(2000));
    } else {
      // Block until unpaired event
      EventBits_t bits = xEventGroupWaitBits(
        eventGroup,
        EVENT_UNPAIRED,
        pdTRUE,   // Clear on exit
        pdFALSE,  // Wait for any bit
        portMAX_DELAY
      );

      // Reset timing after unpair event
      lastWake = xTaskGetTickCount();
    }
  }
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

  // ============================================================================
  // FreeRTOS Initialization (Added for migration)
  // ============================================================================

  // Create queues
  debugQueue = xQueueCreate(20, sizeof(SerialMessage));
  if (debugQueue == NULL) {
    Serial.println("[RTOS] Failed to create debug queue");
    return;
  }

  // Create event group for task synchronization
  eventGroup = xEventGroupCreate();
  if (eventGroup == NULL) {
    Serial.println("[RTOS] Failed to create event group");
    return;
  }

  // Create serial output task (Priority 1 - low)
  BaseType_t taskCreated = xTaskCreate(
    serialOutputTask,   // Task function
    "SerialOut",        // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    1,                  // Priority (low)
    NULL                // Task handle
  );

  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create serial output task");
    return;
  }

  // Create pairing task (Priority 0 - lowest)
  taskCreated = xTaskCreate(
    pairingTask,        // Task function
    "Pairing",          // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    0,                  // Priority (lowest)
    NULL                // Task handle
  );

  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create pairing task");
    return;
  }

  Serial.println("[RTOS] FreeRTOS tasks created successfully");

  // If already paired from saved MAC, signal paired event
  if (paired) {
    xEventGroupSetBits(eventGroup, EVENT_PAIRED);
  }
}

void loop() {
  // Debug mode toggle - NOW HANDLED BY FREERTOS serialOutputTask
  // (Old debug toggle disabled to avoid conflict)
  /*
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
  */

  if (!paired) {
    // Pairing mode - NOW HANDLED BY FREERTOS TASK
    // (Old pairing code disabled to avoid conflict with pairingTask)
    /*
    if (millis() - lastPairAttempt > 2000) {
      lastPairAttempt = millis();
      if (debugMode) {
        Serial.println("[PAIRING] Sending pairing request...");
      }
      esp_now_send(broadcastMAC, (uint8_t*)&pairingData, sizeof(pairingData));
    }
    */
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