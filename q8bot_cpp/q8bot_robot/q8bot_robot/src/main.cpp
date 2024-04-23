#include <Arduino.h>
#include <WiFi.h>
#include <Dynamixel2Arduino.h>
#include <ESPNowW.h>
#include "q8Dynamixel.h"

/*==================================*/
/*========== Definitions ===========*/
/*==================================*/

// ESPNow
char myData[100];
bool incoming = false;
// Dynamixel
HardwareSerial ser(0);
Dynamixel2Arduino q8dxl(ser, 8);
q8Dynamixel q8(q8dxl);
using namespace ControlTableItem;

void onRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // ESPNow callback function to receive data
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
  //          mac_addr[5]);
  // Serial.print("Last Packet Recv from: ");
  // Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Received a packet with size of: ");
  // Serial.println(len);
  Serial.println(myData);
}

/*==================================*/
/*=========== Main Code ============*/
/*==================================*/

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(100);
  }

  Serial.println("q8bot ESPNOW receiver:");
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();
  ESPNow.init();
  ESPNow.reg_recv_cb(onRecv);

  q8.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  q8.moveAll(0.0);
  delay(2000);
  q8.moveAll(45.0);
  delay(2000);
}