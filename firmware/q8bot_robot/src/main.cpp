#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Dynamixel2Arduino.h>

// Q8bot-specific Modules
#include "q8Dynamixel.h"
#include "userParams.h"
#include "systemParams.h"
#include "pinMapping.h"
#include "helpers.h"
#include "communications.h"

// Initialize constants
const uint32_t GAIT_FREQUENCY = 200;
const uint32_t gaitInterval_ms = 1000 / GAIT_FREQUENCY;

// Initialize global objects
esp_now_peer_info_t peerInfo;
HardwareSerial          ser(0);
Dynamixel2Arduino       q8dxl(ser, DXL_DIR_PIN);
q8Dynamixel             q8(q8dxl);

// FreeRTOS Handles
QueueHandle_t rxQueue = NULL;
QueueHandle_t debugQueue = NULL;
EventGroupHandle_t eventGroup = NULL;

// Robot State
volatile RobotState robotState = STATE_UNPAIRED;

MAX1704X FuelGauge = MAX1704X(MAX17043_mV);

// ============================================================================
// ESPNOW Callbacks: ISR-Like, Highest Priority
// ============================================================================
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // Validate length to prevent buffer overrun
  if (len < 1 || len > 250) return;

  // Copy message to queue for processing in espnowRxTask
  ESPNowMessage msg;
  memcpy(msg.mac, mac, 6);
  memcpy(msg.data, data, len);
  msg.len = len;
  msg.timestamp = millis();

  // Non-blocking send - drop message if queue is full
  (void)xQueueSend(rxQueue, &msg, 0);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
}


// ============================================================================
// FreeRTOS Tasks (Ranked by Priority)
// ============================================================================
// FreeRTOS Task: ESP-NOW RX Handler (Priority 3 - HIGHEST)
void espnowRxTask(void* parameter) {
  ESPNowMessage msg;

  while (true) {
    // Wait for messages from onRecv() callback (blocking)
    if (xQueueReceive(rxQueue, &msg, portMAX_DELAY) == pdTRUE) {
      uint8_t msgType = msg.data[0];

      // Handle PAIRING message
      if (msgType == PAIRING && !paired) {
        // Validate PAIRING message length
        if (msg.len < sizeof(PairingMessage)) continue;

        memcpy(&pairingData, msg.data, sizeof(pairingData));
        queuePrint(MSG_DEBUG, "[PAIRING] Pairing request from: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   msg.mac[0], msg.mac[1], msg.mac[2], msg.mac[3], msg.mac[4], msg.mac[5]);

        memcpy(clientMac, msg.mac, 6);
        WiFi.softAPmacAddress(pairingData.macAddr);  // Overwrite with our own MAC
        pairingData.channel = chan;
        pairingData.id = 0;  // Server is ID 0
        addPeer(clientMac);
        esp_now_send(clientMac, (uint8_t*)&pairingData, sizeof(pairingData));
        paired = true;
        lastHeartbeatReceived = millis();

        // Update robot state and event group
        robotState = STATE_PAIRED;
        xEventGroupClearBits(eventGroup, EVENT_UNPAIRED);
        xEventGroupSetBits(eventGroup, EVENT_PAIRED);

        // Save the controller MAC address to EEPROM
        storage.savePeerMAC(clientMac);
        queuePrint(MSG_DEBUG, "[STORAGE] Saved controller MAC to EEPROM\n");
        queuePrint(MSG_INFO, "[PAIRING] Paired successfully\n");
      }
      // Handle HEARTBEAT message
      else if (msgType == HEARTBEAT && paired) {
        // Validate HEARTBEAT message length
        if (msg.len < sizeof(HeartbeatMessage)) continue;

        lastHeartbeatReceived = millis();
        queuePrint(MSG_DEBUG, "[HEARTBEAT] Received, echoing back\n");

        // Echo heartbeat back to controller
        esp_now_send(msg.mac, msg.data, msg.len);
      }
      // Handle DATA message
      else if (msgType == DATA && paired) {
        q8.handleDataMessage(msg);
      }
    }
  }
}

// FreeRTOS Task: Heartbeat Monitor (Priority 2)
void heartbeatMonitorTask(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    // Only monitor heartbeat when paired
    if (paired) {
      unsigned long now = millis();

      // Avoid underflow: only check timeout if now >= lastHeartbeatReceived
      if (now >= lastHeartbeatReceived) {
        unsigned long timeSinceLastMsg = now - lastHeartbeatReceived;

        // Check for timeout (only in auto-pairing mode)
#ifndef PERMANENT_PAIRING_MODE
        if (timeSinceLastMsg > HEARTBEAT_TIMEOUT_ROBOT) {
          queuePrint(MSG_DEBUG, "[HEARTBEAT] Timeout detected (%lums since last message)\n", timeSinceLastMsg);
          q8.toggleTorque(0);        // Disable torque hardware
          q8.resetTorqueState();     // Sync internal flag to match disabled state
          unpair();
        }
#endif
      }
    }

    // Check every 1 second
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
  }
}

// FreeRTOS Task: Robot State Manager (Priority 1)
void robotStateTask(void* parameter) {
  TickType_t lastStateChange = xTaskGetTickCount();
  unsigned long lastActivity = 0;
  bool firstRun = true;  // Check saved MAC on first iteration

  // Give other tasks time to start (especially serialOutputTask)
  vTaskDelay(pdMS_TO_TICKS(100));

  while (true) {
    // On first run, check for saved MAC address
    if (firstRun) {
      firstRun = false;
      if (storage.loadPeerMAC(clientMac)) {
        queuePrint(MSG_DEBUG, "[PAIRING] Found saved controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   clientMac[0], clientMac[1], clientMac[2], clientMac[3], clientMac[4], clientMac[5]);
        addPeer(clientMac);
        paired = true;
        robotState = STATE_PAIRED;
        lastHeartbeatReceived = millis();
        xEventGroupClearBits(eventGroup, EVENT_UNPAIRED);
        xEventGroupSetBits(eventGroup, EVENT_PAIRED);
        queuePrint(MSG_INFO, "[PAIRING] Attempting to reconnect to saved controller\n");
      } else {
        queuePrint(MSG_INFO, "[PAIRING] No saved MAC found - waiting for pairing request\n");
      }
    }

    unsigned long now = millis();
    RobotState currentState = robotState;

    switch (currentState) {
      case STATE_UNPAIRED: {
        // Slow blink every 2 seconds
        if (now - lastActivity >= 2000) {
          lastActivity = now;
          digitalWrite(LED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(200));
          digitalWrite(LED_PIN, LOW);
          queuePrint(MSG_DEBUG, "[PAIRING] Waiting for pairing...\n");
        }
        break;
      }

      case STATE_PAIRED: {
        // Double blink every 2 seconds (waiting for torque enable)
        if (now - lastActivity >= 2000) {
          lastActivity = now;
          for (int i = 0; i < 2; i++) {
            digitalWrite(LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(200));
            digitalWrite(LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(300));
          }

          // Try to start robot (enable torque)
          if (q8.commStart()) {
            robotState = STATE_STARTED;
            started = true;
            xEventGroupSetBits(eventGroup, EVENT_STARTED);
            queuePrint(MSG_INFO, "[STATE] Robot started!\n");
            lastActivity = now;  // Reset for breathing pattern
          } else {
            queuePrint(MSG_DEBUG, "[STATE] Waiting for robot start...\n");
          }
        }
        break;
      }

      case STATE_STARTED: {
        // Breathing LED pattern every 10 seconds
        if (now - lastActivity >= 10000) {
          lastActivity = now;

          for (int brightness = 0; brightness <= 255; brightness += 2) {
            analogWrite(LED_PIN, brightness);
            vTaskDelay(pdMS_TO_TICKS(2));
          }
          for (int brightness = 255; brightness >= 0; brightness -= 2) {
            analogWrite(LED_PIN, brightness);
            vTaskDelay(pdMS_TO_TICKS(2));
          }
        }
        break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Check state every 100ms
  }
}

// FreeRTOS Task: Serial Output Handler (Priority 1)
void serialOutputTask(void* parameter) {
  SerialMessage msg;

  while (true) {
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

    // Check for incoming serial commands
#ifdef PERMANENT_PAIRING_MODE
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'p') {
        queuePrint(MSG_INFO, "[PAIRING] Force pairing mode requested\n");
        q8.toggleTorque(0);        // Disable torque hardware
        q8.resetTorqueState();     // Sync internal flag to match disabled state
        unpair();
      } else if (c == 'd') {
        debugMode = !debugMode;
        queuePrint(MSG_INFO, "Debug mode: %s\n", debugMode ? "ON" : "OFF");
      }
    }
#endif

    // Low priority - yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void gaitTask(void* parameter) {
  while (true) {
    unsigned long timeSinceLastMsg = millis() - lastHeartbeatReceived;
    if (q8.getGaitActive() && timeSinceLastMsg < HEARTBEAT_TIMEOUT_ROBOT) {
      q8.performNextGaitMove();
    }

    vTaskDelay(pdMS_TO_TICKS(gaitInterval_ms));
  }
}

// ============================================================================
// Arduino Setup. Configures FreeRTOS tasks, ESP-NOW, and other hardware.
// ============================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
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
    "SerialOutput",     // Task name
    2048,               // Stack size (bytes)
    NULL,               // Parameters
    1,                  // Priority (low)
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
    4096,               // Stack size (bytes) - larger for message processing
    NULL,               // Parameters
    3,                  // Priority (high - processes incoming messages)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create ESP-NOW RX task");
    initSuccess = false;
  }

  taskCreated = xTaskCreate(
    gaitTask,             // Task function
    "Gait",               // Task name
    4096,                 // Stack size (bytes) - larger for gait logic
    NULL,                 // Parameters
    4,                    // Priority (max - manages movement)
    NULL                  // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create gait task");
    initSuccess = false;
  }

  // Create heartbeat monitor task (Priority 2)
  taskCreated = xTaskCreate(
    heartbeatMonitorTask, // Task function
    "HeartbeatMonitor",   // Task name
    2048,                 // Stack size (bytes)
    NULL,                 // Parameters
    2,                    // Priority (medium - monitors connection)
    NULL                  // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create heartbeat monitor task");
    initSuccess = false;
  }

  // Create robot state manager task (Priority 1)
  taskCreated = xTaskCreate(
    robotStateTask,     // Task function
    "RobotState",       // Task name
    4096,               // Stack size (bytes) - larger for LED patterns and state logic
    NULL,               // Parameters
    1,                  // Priority (low - manages LED patterns)
    NULL                // Task handle
  );
  if (taskCreated != pdPASS) {
    Serial.println("[RTOS] Failed to create robot state task");
    initSuccess = false;
  }

  // Check if all FreeRTOS initialization succeeded
  if (!initSuccess) {
    Serial.println("[RTOS] Initialization failed - halting system");
    while(1) { delay(1000); }  // Halt system indefinitely
  }

  // Init Wi-Fi
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("esp-server", nullptr, chan); // Optional, just to enable softAP mode
  chan = WiFi.channel();
  WiFi.softAPmacAddress(serverMac);

  // Init ESP-NOW (after FreeRTOS primitives are ready)
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onRecv);  // Set up callback when data is received.

  // MAX17043 Init
  if (FuelGauge.begin()){
    FuelGauge.reset(); // Reset the device.
    delay(250);
    FuelGauge.quickstart();
    delay(125);
    displayReading(); // Display an initial reading.
  } else{
    Serial.println("[ROBOT] MAX17043 NOT found. Continuing\n");
  }

  // Initialize Dynamixel object
  q8.begin();
}

// Loop does nothing - all work done in FreeRTOS tasks
void loop() {
  delay(1);
}