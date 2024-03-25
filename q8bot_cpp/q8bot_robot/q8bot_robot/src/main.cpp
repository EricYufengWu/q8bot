#include <Arduino.h>
#include <WiFi.h>
#include "ESPNowW.h"

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  float pos[8];
  uint16_t dur;
  bool torque = false;
} struct_message;

// Create a struct_message called myData
// struct_message myData;
char myData[100];

// Flag for knowing whether there's incoming data
bool incoming = false;

// Enable torque: toggle the torqueOn register to all DXLs. Get prevPos from current joint reading.
void enableTorque(){

}

void onRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
  //          mac_addr[5]);
  // Serial.print("Last Packet Recv from: ");
  // Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Received a packet with size of: ");
  // Serial.println(len);

  // for (int i = 0; i < 8; i++){
  //   Serial.print(myData.pos[i]);
  //   Serial.print(" ");
  // }
  // Serial.print(myData.dur);
  // Serial.print(" ");
  // Serial.println(myData.torque);

  Serial.println(myData);
}

void q8_move(){

}

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(50);
  }
  Serial.println("ESPNow receiver Demo");
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();
  ESPNow.init();
  ESPNow.reg_recv_cb(onRecv);
}

void loop() {
  // Serial.println("Looping");
  delay(10);
}