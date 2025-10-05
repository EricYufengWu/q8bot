#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

// ESP-NOW Messaging Types
enum MsgType : uint8_t{
  PAIRING,
  DATA,
  HEARTBEAT,
};
struct PairingMessage{
  uint8_t msgType = PAIRING;
  uint8_t id;
  uint8_t macAddr[6];
  uint8_t channel;
};
struct CharMessage{
  uint8_t msgType = DATA;
  uint8_t id;
  char data[100];
};
struct IntMessage{
  uint8_t msgType = DATA;
  uint8_t id;
  uint16_t data[100];
};
struct HeartbeatMessage{
  uint8_t msgType = HEARTBEAT;
  uint8_t id;
  uint32_t timestamp;
};

// ESP-NOW Comms
PairingMessage pairingData;
CharMessage sendMsg;
IntMessage recvMsg;
HeartbeatMessage heartbeatMsg;
int chan = 1;  // Must be the same(similar) across server and client
bool paired = false;
uint8_t serverMac[6];
uint8_t clientMac[6];  // This device
uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long lastPairAttempt;

// Heartbeat tracking
unsigned long lastHeartbeatSent = 0;
unsigned long lastHeartbeatReceived = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000;      // Send every 5s
const unsigned long HEARTBEAT_TIMEOUT = 15000;      // Unpair after 15s no response

// Debug mode
bool debugMode = false;

// ============================================================================
// FreeRTOS Data Structures (Added for FreeRTOS migration - not yet used)
// ============================================================================

// Message queue structure for ISR → RX Handler
typedef struct {
  uint8_t mac[6];
  uint8_t data[250];
  int len;
  uint32_t timestamp;
} ESPNowMessage;

// Serial output message types
enum SerialMsgType : uint8_t {
  MSG_DEBUG,  // Can be toggled off with 'd' command
  MSG_INFO    // Always printed (system events)
};

typedef struct {
  SerialMsgType type;
  char text[128];
} SerialMessage;

// Lock-free shared state
typedef struct {
  volatile bool paired;
  uint8_t serverMac[6];
  volatile uint32_t lastHeartbeatRx;
  bool debugMode;
} ControllerState;

// FreeRTOS Handles (to be initialized in setup)
extern QueueHandle_t rxQueue;
extern QueueHandle_t debugQueue;
extern QueueHandle_t dataOutputQueue;
extern EventGroupHandle_t eventGroup;

// Event group bits
#define EVENT_PAIRED        (1 << 0)
#define EVENT_UNPAIRED      (1 << 1)
#define EVENT_HEARTBEAT_RX  (1 << 2)