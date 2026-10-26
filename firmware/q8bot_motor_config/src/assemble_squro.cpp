#include <Arduino.h>
#include <HardwareSerial.h>
#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;
HardwareSerial motorSerial(0);
Dynamixel2Arduino dxl(motorSerial, 8);
#include "head_config.h"
#include "squro_assembly.h"

void setup() {
  Serial.begin(115200);
  delay(1500);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  SquroAssembly::startupRelease();
  Serial.println("SQuRo bench assembly: no automatic movement; startup releases powered servos.");
  Serial.println("CENTERHEAD: ID19 -> 4096. ALIGNFL/ALIGNFR: front elbow/shoulder -> 4695/5564.");
  Serial.println("ALIGNBL/ALIGNBR: rear elbow/shoulder -> 3627/4745. Outer rear servos must be ID16/18.");
  Serial.println("Confirm physical mapping and detach the selected pair's linkage before alignment.");
  Serial.println("Send KEEP at least every 10 seconds while active. RELEASE or ! stops. Hold limit: 5 minutes.");
}

void loop() {
  while (Serial.available()) SquroAssembly::consume(static_cast<char>(Serial.read()));
  SquroAssembly::tick();
  delay(2);
}
