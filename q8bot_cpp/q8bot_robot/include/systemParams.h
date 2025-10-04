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
// uint16_t myData[100];
// char theirData[100];
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
const unsigned long HEARTBEAT_TIMEOUT_ROBOT = 20000;  // Unpair after 20s no heartbeat from controller

// Debug mode
bool debugMode = false;