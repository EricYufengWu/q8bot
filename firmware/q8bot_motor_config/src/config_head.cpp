// Head-servo configuration only: no torque-on or commanded movement.
// Configures ID 19 on the daisy chain while leaving IDs 11-18 untouched.
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;
HardwareSerial motorSerial(0);
Dynamixel2Arduino dxl(motorSerial, 8);
#include "head_config.h"

void setup() {
  Serial.begin(115200);
  delay(1500);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  Serial.println("Q8bot head servo setup. No automatic writes or movement.");
  Serial.println("CONFIGHEAD configures ID19; STATUS reads the head; LEGS reads IDs 11-18.");
  Serial.println("Keep all eight leg servos connected, with only one new head servo on the bus.");
}

void loop() {
  static char command[16];
  static uint8_t used = 0;
  static bool overflow = false;
  static uint32_t lastScan = 0;
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      command[used] = '\0';
      if (!overflow && strcmp(command, "CONFIGHEAD") == 0) HeadConfig::configure();
      else if (!overflow && strcmp(command, "STATUS") == 0) HeadConfig::status();
      else if (!overflow && strcmp(command, "LEGS") == 0) HeadConfig::legStatus();
      else if (used) Serial.println("Commands: CONFIGHEAD, STATUS, LEGS");
      used = 0;
      overflow = false;
    } else if (!overflow && used < sizeof(command) - 1) command[used++] = c;
    else overflow = true;
  }
  if (millis() - lastScan >= 5000) {
    lastScan = millis();
    HeadConfig::Candidate candidate{};
    uint8_t found = HeadConfig::findCandidate(candidate);
    Serial.printf("[scan] %u head candidate(s) at ID1/ID19.\n", found);
  }
}
