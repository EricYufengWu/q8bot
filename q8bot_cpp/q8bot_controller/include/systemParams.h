#include <Arduino.h>

// ESP-NOW Messaging Types
enum MsgType : uint8_t{
  PAIRING,
  DATA,
};
struct PairingMessage{
  uint8_t msgType;
  uint8_t id;
  uint8_t macAddr[6];
  uint8_t channel;
};
struct DataMessage{
  uint8_t msgType;
  uint8_t id;
  char data[100];
};

// ESP-NOW Comms
PairingMessage pairingData;
DataMessage dataMsg;
int chan = 1;  // Must be the same(similar) across server and client
bool paired = false;
uint8_t serverMac[6];
uint8_t clientMac[6];  // This device
uint8_t broadcastMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned long lastPairAttempt;