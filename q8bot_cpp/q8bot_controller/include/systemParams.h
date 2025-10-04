#include <Arduino.h>

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