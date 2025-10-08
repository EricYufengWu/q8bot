#include <arduino.h>

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
struct DataMessage{
  uint8_t msgType = DATA;
  uint8_t id;
  uint16_t data[100];
};
struct HeartbeatMessage{
  uint8_t msgType = HEARTBEAT;
  uint8_t id;
  uint32_t timestamp;
};

// Dynamixel Variables
bool recordData = false;
size_t masterSize;
size_t smallerSize = 4;
uint16_t* rData = nullptr;

// MAX17043 Variables
float raw;
char intStr[10];

// ESP-Now Comms
bool incoming = false;
PairingMessage pairingData;
CharMessage theirMsg;
DataMessage myMsg;
int chan = 1;
int paired = false;
uint8_t clientMac[6];  // This is the controller MCU.
uint8_t serverMac[6];  // This us. The robot is the server.

// Heartbeat tracking (robot side)
unsigned long lastHeartbeatReceived = 0;
const unsigned long HEARTBEAT_TIMEOUT_ROBOT = 5000;  // Unpair after 20s no heartbeat from controller

// Debug mode
bool debugMode = false;

// FreeRTOS Handles (to be initialized in setup)
extern QueueHandle_t rxQueue;
extern QueueHandle_t debugQueue;
extern EventGroupHandle_t eventGroup;

// FreeRTOS Event Bits
#define EVENT_PAIRED    (1 << 0)
#define EVENT_UNPAIRED  (1 << 1)
#define EVENT_STARTED   (1 << 2)

// Robot State Machine
enum RobotState : uint8_t {
  STATE_UNPAIRED,      // Waiting for controller pairing
  STATE_PAIRED,        // Paired but torque not enabled
  STATE_STARTED        // Torque enabled, robot operational
};

extern volatile RobotState robotState;

// FreeRTOS Message Structures
struct ESPNowMessage {
  uint8_t mac[6];
  uint8_t data[250];
  int len;
  uint32_t timestamp;
};

enum SerialMsgType : uint8_t {
  MSG_INFO,
  MSG_DEBUG
};

struct SerialMessage {
  SerialMsgType type;
  char text[128];
};