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

// FreeRTOS Handles
QueueHandle_t rxQueue = NULL;
QueueHandle_t debugQueue = NULL;
QueueHandle_t dataOutputQueue = NULL;
EventGroupHandle_t eventGroup = NULL;


// ============================================================================
// Helper Functions
// ============================================================================
void queuePrint(SerialMsgType type, const char* format, ...) {
  SerialMessage msg;
  msg.type = type;

  va_list args;
  va_start(args, format);
  vsnprintf(msg.text, sizeof(msg.text), format, args);
  va_end(args);

  xQueueSend(debugQueue, &msg, 0);
}

bool addPeer(const uint8_t* mac) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = chan;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

void unpair() {
  queuePrint(MSG_DEBUG, "[HEARTBEAT] Connection lost - returning to pairing mode\n");

  esp_now_del_peer(serverMac);
  storage.clearPeerMAC();
  queuePrint(MSG_DEBUG, "[STORAGE] Cleared peer MAC from EEPROM\n");

  memset(serverMac, 0, sizeof(serverMac));
  paired = false;
  lastPairAttempt = millis();

  // Signal pairing task to resume broadcasting (if FreeRTOS is running)
  if (eventGroup != NULL) {
    xEventGroupSetBits(eventGroup, EVENT_UNPAIRED);
  }
}


// ============================================================================
// ESPNOW Callbacks: ISR-Like, Highest Priority
// ============================================================================
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // Validate minimum length
  if (len < 1 || len > 250) return;

  // Copy to queue structure
  ESPNowMessage msg;
  memcpy(msg.mac, mac, 6);
  memcpy(msg.data, data, len);
  msg.len = len;
  msg.timestamp = millis();

  // Non-blocking send; drop if full
  (void)xQueueSend(rxQueue, &msg, 0);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}

// ============================================================================
// FreeRTOS Tasks (Ranked by Priority)
// ============================================================================
// FreeRTOS Task: Command Forwarding (Priority 4 - HIGHEST)
void commandForwardingTask(void *param) {
  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    if (Serial.available()) {
      char c = Serial.peek();

      if (c == 'd') {
        Serial.read();
        debugMode = !debugMode;
        queuePrint(MSG_INFO, "Debug mode: %s\n", debugMode ? "ON" : "OFF");
      }
#ifdef PERMANENT_PAIRING_MODE
      else if (c == 'p') {
        Serial.read();
        queuePrint(MSG_DEBUG, "[PAIRING] Force pairing mode requested\n");
        unpair();
      }
#endif
      else if (paired) {
        // Forward joint commands to robot
        int bytesRead = Serial.readBytesUntil(';', sendMsg.data, sizeof(sendMsg.data) - 1);
        if (bytesRead > 0) {
          sendMsg.data[bytesRead] = '\0';

          // Send to robot's MAC address
          sendMsg.msgType = DATA;
          sendMsg.id = 1;
          esp_now_send(serverMac, (uint8_t*)&sendMsg, sizeof(sendMsg));
        }
      }
    }

    // Run at 250 Hz (4ms period)
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(4));
  }
}

// FreeRTOS Task: ESP-NOW RX Handler (Priority 3)
void espnowRxTask(void *param) {
  ESPNowMessage msg;

  while (1) {
    // Block waiting for messages from ISR queue
    if (xQueueReceive(rxQueue, &msg, portMAX_DELAY) == pdTRUE) {

      // Process based on message type
      if (msg.data[0] == PAIRING) {
        // Validate PAIRING message length
        if (msg.len < sizeof(PairingMessage)) continue;

        memcpy(&pairingData, msg.data, sizeof(PairingMessage));
        queuePrint(MSG_DEBUG, "[PAIRING] Paired with server: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   msg.mac[0], msg.mac[1], msg.mac[2], msg.mac[3], msg.mac[4], msg.mac[5]);

        memcpy(serverMac, msg.mac, sizeof(serverMac));
        addPeer(serverMac);
        paired = true;
        lastHeartbeatReceived = millis();

        // Save the MAC address to EEPROM
        storage.savePeerMAC(serverMac);
        queuePrint(MSG_DEBUG, "[STORAGE] Saved peer MAC to EEPROM\n");
        queuePrint(MSG_DEBUG, "[HEARTBEAT] Connection established, heartbeat timer started\n");

        // Signal pairing task to stop broadcasting
        if (eventGroup != NULL) {
          xEventGroupSetBits(eventGroup, EVENT_PAIRED);
        }

      } else if (msg.data[0] == HEARTBEAT) {
        // Robot echoed heartbeat back
        if (msg.len < sizeof(HeartbeatMessage)) continue;

        lastHeartbeatReceived = millis();

        HeartbeatMessage hbMsg;
        memcpy(&hbMsg, msg.data, sizeof(HeartbeatMessage));
        uint32_t rtt = millis() - hbMsg.timestamp;
        queuePrint(MSG_DEBUG, "[HEARTBEAT] ACK received, RTT: %ums\n", rtt);

      } else if (msg.data[0] == DATA) {
        // Validate DATA message length
        if (msg.len < sizeof(IntMessage)) continue;

        memcpy(&recvMsg, msg.data, sizeof(IntMessage));
        lastHeartbeatReceived = millis();  // Any DATA also counts as "alive"

        // Print data array
        for (int i = 0; i < 100; i++) {
          Serial.print(recvMsg.data[i]);
          Serial.print(" ");
        }
        Serial.println();
        memset(&recvMsg, 0, sizeof(recvMsg));
      }
    }
  }
}

// FreeRTOS Task: Heartbeat Manager (Priority 2)
void heartbeatTask(void *param) {
  TickType_t lastWake = xTaskGetTickCount();

  while (1) {
    // Only send heartbeat when paired
    if (paired) {
      // Send heartbeat
      heartbeatMsg.msgType = HEARTBEAT;
      heartbeatMsg.id = 1;
      heartbeatMsg.timestamp = millis();
      lastHeartbeatSent = millis();

      unsigned long timeSinceLastHB = millis() - lastHeartbeatReceived;
      queuePrint(MSG_DEBUG, "[HEARTBEAT] Sending heartbeat (last response: %lums ago)\n", timeSinceLastHB);

      esp_now_send(serverMac, (uint8_t*)&heartbeatMsg, sizeof(heartbeatMsg));

      // Check for timeout (only in auto-pairing mode)
#ifndef PERMANENT_PAIRING_MODE
      if (timeSinceLastHB > HEARTBEAT_TIMEOUT) {
        queuePrint(MSG_DEBUG, "[HEARTBEAT] Timeout detected (%lums since last response)\n", timeSinceLastHB);
        unpair();
      }
#endif
    }

    // Run at frequency defined by constant
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(HEARTBEAT_INTERVAL));
  }
}

// FreeRTOS Task: Serial Output / Debug (Priority 1)
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

    // Low priority - yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// FreeRTOS Task: Pairing Manager (Priority 0 - LOWEST)
void pairingTask(void *param) {
  TickType_t lastWake = xTaskGetTickCount();

  // On first run, check for saved MAC address
  if (storage.loadPeerMAC(serverMac)) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            serverMac[0], serverMac[1], serverMac[2],
            serverMac[3], serverMac[4], serverMac[5]);
    queuePrint(MSG_DEBUG, "[PAIRING] Found saved MAC: %s\n", macStr);

    addPeer(serverMac);
    paired = true;
    lastHeartbeatReceived = millis();
    lastHeartbeatSent = millis();

    queuePrint(MSG_DEBUG, "[PAIRING] Attempting to reconnect to saved peer\n");

    // Signal that we're already paired
    xEventGroupSetBits(eventGroup, EVENT_PAIRED);
  } else {
    queuePrint(MSG_DEBUG, "[PAIRING] No saved MAC found - entering pairing mode\n");
  }

  while (1) {
    // Check pairing state (use old global for now, will migrate to atomic later)
    bool isPaired = paired;

    if (!isPaired) {
      // Send pairing broadcast every 2s
      if (debugQueue != NULL) {
        queuePrint(MSG_DEBUG, "[PAIRING] Sending broadcast...\n");
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


// ============================================================================
// Arduino Setup. Configures ESP-NOW and FreeRTOS tasks.
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  // delay(2000);  // Useful for debugging

  bool initSuccess = true;

  // FreeRTOS Initialization
  // Create queues
  rxQueue = xQueueCreate(10, sizeof(ESPNowMessage));
  if (rxQueue == NULL) {
    Serial.println("[RTOS] Failed to create RX queue");
    initSuccess = false;
  }

  debugQueue = xQueueCreate(20, sizeof(SerialMessage));
  if (debugQueue == NULL) {
    Serial.println("[RTOS] Failed to create debug queue");
    initSuccess = false;
  }

  // Create event group for task synchronization
  eventGroup = xEventGroupCreate();
  if (eventGroup == NULL) {
    Serial.println("[RTOS] Failed to create event group");
    initSuccess = false;
  }

  // Create FreeRTOS tasks
  // Create serial output task (Priority 1)
  BaseType_t taskCreated = xTaskCreate(
    serialOutputTask,   // Task function
    "SerialOut",        // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    1,                  // Priority (low - non-critical output)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create serial output task");
    initSuccess = false;
  }

  // Create ESP-NOW RX handler task (Priority 3)
  taskCreated = xTaskCreate(
    espnowRxTask,       // Task function
    "ESPNowRX",         // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    3,                  // Priority (high - process messages quickly)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create ESP-NOW RX task");
    initSuccess = false;
  }

  // Create heartbeat manager task (Priority 2)
  taskCreated = xTaskCreate(
    heartbeatTask,      // Task function
    "Heartbeat",        // Task name
    3072,               // Stack size (bytes)
    NULL,               // Parameters
    2,                  // Priority (medium - send/monitor heartbeats)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create heartbeat task");
    initSuccess = false;
  }

  // Create command forwarding task (Priority 4 - HIGHEST)
  taskCreated = xTaskCreate(
    commandForwardingTask, // Task function
    "CmdFwd",              // Task name
    3072,                  // Stack size (bytes)
    NULL,                  // Parameters
    4,                     // Priority (highest - 200 Hz critical timing)
    NULL                   // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create command forwarding task");
    initSuccess = false;
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
    initSuccess = false;
  }

  // Check if all FreeRTOS initialization succeeded
  if (!initSuccess) {
    Serial.println("[RTOS] Initialization failed - halting system");
    while(1) { delay(1000); }  // Halt system indefinitely
  }

  // Init Wi-Fi; set device as a Wi-Fi Station (after FreeRTOS primitives are ready)
  WiFi.mode(WIFI_STA);
  WiFi.macAddress(clientMac);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Init ESP-NOW (after FreeRTOS primitives are ready)
  if (esp_now_init() != ESP_OK) {
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(OnDataSent);
  addPeer(broadcastMAC);
}

// Loop does nothing - all work done in FreeRTOS tasks
void loop() {
  delay(1);
}