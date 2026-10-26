// Replacement-only configuration: no torque-on or position commands.
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;
HardwareSerial motorSerial(0);
Dynamixel2Arduino dxl(motorSerial, 8);
#include "replacement_config.h"
#ifdef Q8_ENABLE_ALIGNMENT
#include "replacement_alignment.h"
#endif

void setup() {
  Serial.begin(115200);
  delay(1500);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
#ifdef Q8_ENABLE_ALIGNMENT
  Alignment18::torqueOff();
  Serial.println("Q8bot ID18 mounting alignment. No automatic movement.");
  Serial.println("ALIGN18 moves and holds ID18; RELEASE18 or ! releases it.");
#else
  Serial.println("Q8bot replacement ID18 setup. No automatic writes or movement.");
#endif
  Serial.println("CONFIG18 configures the replacement; STATUS reads current state.");
  Serial.println("Leave its linkage detached until mounting-position alignment.");
}

void loop() {
  static char command[16];
  static uint8_t used = 0;
  static bool overflow = false;
  static uint32_t lastStatus = 0;
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
#ifdef Q8_ENABLE_ALIGNMENT
    if (c == '!') {
      Alignment18::release(); used = 0; overflow = false; continue;
    }
#endif
    if (c == '\r') continue;
    if (c == '\n') {
      command[used] = '\0';
#ifdef Q8_ENABLE_ALIGNMENT
      if (!overflow && strcmp(command, "RELEASE18") == 0) Alignment18::release();
      else if (!overflow && strcmp(command, "ALIGN18") == 0) Alignment18::begin();
      else if (Alignment18::active) Serial.println("ALIGN18 active; release before configuration or a full scan.");
      else
#endif
      if (!overflow && strcmp(command, "CONFIG18") == 0) Replacement18::configure();
      else if (!overflow && strcmp(command, "STATUS") == 0) Replacement18::status();
      used = 0;
      overflow = false;
    } else if (!overflow && used < sizeof(command) - 1) command[used++] = c;
    else overflow = true;
  }
#ifdef Q8_ENABLE_ALIGNMENT
  Alignment18::tick();
  bool canScan = !Alignment18::active;
#else
  bool canScan = true;
#endif
  if (canScan && millis() - lastStatus >= 5000) {
    lastStatus = millis();
    Replacement18::status();
  }
  delay(5);
}
