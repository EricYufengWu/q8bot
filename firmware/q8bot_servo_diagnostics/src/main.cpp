// Default build: read-only diagnostics. Optional test builds are command-gated.
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;
HardwareSerial motorSerial(0);
Dynamixel2Arduino dxl(motorSerial, 8);

#ifdef Q8_ENABLE_ID18_TEST
#include "id18_test.h"
#endif

void readItem(uint8_t id, uint8_t item, const char* name, float scale = 1.0f) {
  int32_t value = dxl.readControlTableItem(item, id, 30);
  auto libError = dxl.getLastLibErrCode();
  uint8_t status = dxl.getLastStatusPacketError();
  if (libError != 0 || (status & 0x7f) != 0) {
    Serial.printf("  %s: READ FAILED (library=%d, status=0x%02X)\n", name, libError, status);
    return;
  }
  if (scale == 1.0f) Serial.printf("  %s: %ld", name, static_cast<long>(value));
  else Serial.printf("  %s: %.2f", name, value * scale);
  if (status & 0x80) Serial.print(" [SERVO HARDWARE ALERT]");
  Serial.println();
  if (item == HARDWARE_ERROR_STATUS && value != 0) {
    if (value & 0x01) Serial.println("    Input voltage error");
    if (value & 0x04) Serial.println("    Overheating error");
    if (value & 0x10) Serial.println("    Electrical shock / insufficient power error");
    if (value & 0x20) Serial.println("    Persistent overload error");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
#ifdef Q8_ENABLE_ID18_TEST
  // A controller reset during a test must not leave the tested servo holding.
  Id18Test::torqueOff();
  Serial.printf("Q8bot ID%u test firmware. No automatic movement.\n", Id18Test::ID);
  Serial.printf("Send %s on its own line for a bounded test; ! aborts.\n", Id18Test::COMMAND);
#else
  Serial.println("Q8bot read-only servo diagnostics: IDs 11-18 at 1 Mbps.");
  Serial.println("No torque, position, mode, ID or baud-rate writes are sent.");
#endif
  Serial.println("Power-cycling may have cleared the original fault flags.");
}

void loop() {
#ifdef Q8_ENABLE_ID18_TEST
  static uint32_t lastScan = 0;
  pollId18TestCommand();
  if (millis() - lastScan < 5000) {
    delay(5);
    return;
  }
  lastScan = millis();
#endif
  Serial.println("\n=== Servo scan ===");
  for (uint8_t id = 11; id <= 18; ++id) {
    Serial.printf("ID %u%s\n", id, id == 18 ? " (reported faulty)" : "");
    readItem(id, HARDWARE_ERROR_STATUS, "Hardware error flags");
    readItem(id, PRESENT_INPUT_VOLTAGE, "Supply voltage (V)", 0.1f);
    readItem(id, PRESENT_TEMPERATURE, "Temperature (C)");
    readItem(id, PRESENT_CURRENT, "Current (mA)");
    readItem(id, TORQUE_ENABLE, "Torque enabled");
    readItem(id, PRESENT_POSITION, "Present position (ticks)");
    readItem(id, GOAL_POSITION, "Goal position (ticks)");
    readItem(id, DRIVE_MODE, "Drive mode");
    readItem(id, OPERATING_MODE, "Operating mode");
    readItem(id, HOMING_OFFSET, "Homing offset");
    readItem(id, PROFILE_VELOCITY, "Profile duration (ms, if time-based)");
    readItem(id, PROFILE_ACCELERATION, "Profile acceleration time (ms, if time-based)");
    readItem(id, SHUTDOWN, "Shutdown mask");
    readItem(id, POSITION_P_GAIN, "Position P gain");
    readItem(id, POSITION_I_GAIN, "Position I gain");
    readItem(id, POSITION_D_GAIN, "Position D gain");
    readItem(id, PWM_LIMIT, "PWM limit");
    readItem(id, GOAL_PWM, "Goal PWM / output cap");
    readItem(id, CURRENT_LIMIT, "Current limit (mA)");
  }
#ifndef Q8_ENABLE_ID18_TEST
  delay(5000);
#endif
}
