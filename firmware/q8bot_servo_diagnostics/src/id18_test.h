// Optional, explicitly commanded single-servo test. The default build excludes it.
#pragma once
#include <stdint.h>
#include <string.h>

namespace Id18Test {
#ifndef Q8_TEST_ID
#define Q8_TEST_ID 18
#endif
constexpr uint8_t ID = Q8_TEST_ID;
static_assert(ID == 18 || ID == 16, "Only the two tested rear-joint configurations are supported.");
constexpr const char* COMMAND = ID == 18 ? "TEST18" : "TEST16";
constexpr int32_t MAX_PWM = 133;  // About 15% duty, not a current/torque limit.
constexpr int32_t STEP_TICKS = 34;  // About 3 degrees, toward the earlier position.
constexpr uint32_t LEG_MS = 1400;

bool read(uint8_t item, int32_t& value, uint8_t id = ID) {
  value = dxl.readControlTableItem(item, id, 30);
  return dxl.getLastLibErrCode() == 0 && dxl.getLastStatusPacketError() == 0;
}

bool write(uint8_t item, int32_t value) {
  // No mode, calibration, ID, persistent limit, or shutdown-mask writes.
  if (item != TORQUE_ENABLE && item != GOAL_PWM &&
      item != PROFILE_VELOCITY && item != PROFILE_ACCELERATION &&
      item != GOAL_POSITION) return false;
  if (!dxl.writeControlTableItem(item, ID, value, 30) ||
      dxl.getLastLibErrCode() != 0 || dxl.getLastStatusPacketError() != 0) return false;
  int32_t actual;
  return read(item, actual) && actual == value;
}

bool torqueOff() {
  // Attempt every time, including after a hardware alert; verify torque itself.
  for (int n = 0; n < 3; ++n) {
    dxl.writeControlTableItem(static_cast<uint8_t>(TORQUE_ENABLE), ID, int32_t{0}, uint32_t{30});
    int32_t torque = dxl.readControlTableItem(static_cast<uint8_t>(TORQUE_ENABLE), ID, uint32_t{30});
    if (dxl.getLastLibErrCode() == 0 &&
        (dxl.getLastStatusPacketError() & 0x7f) == 0 && torque == 0) return true;
  }
  Serial.println("SERVO TORQUE-OFF NOT CONFIRMED: SWITCH BATTERY POWER OFF.");
  return false;
}

struct Sample {
  int32_t position, current, pwm, voltage, temperature, torque, fault;
};

bool sample(Sample& s) {
  return read(HARDWARE_ERROR_STATUS, s.fault) &&
         read(PRESENT_POSITION, s.position) && read(PRESENT_CURRENT, s.current) &&
         read(PRESENT_PWM, s.pwm) && read(PRESENT_INPUT_VOLTAGE, s.voltage) &&
         read(PRESENT_TEMPERATURE, s.temperature) && read(TORQUE_ENABLE, s.torque);
}

bool conditionsOk(const Sample& s, int32_t start, int32_t pwmCap) {
  return s.fault == 0 && s.voltage >= 35 && s.voltage <= 55 &&
         s.temperature < 45 && s.current >= -250 && s.current <= 250 &&
         s.pwm >= -pwmCap && s.pwm <= pwmCap &&
         s.position >= start - 90 && s.position <= start + 90;
}

bool observe(int32_t start, int32_t pwmCap, const char* stage) {
  const uint32_t began = millis();
  while (millis() - began < LEG_MS) {
    while (Serial.available()) {
      if (Serial.read() == '!') {
        Serial.println("SERVO TEST ABORT: requested stop.");
        return false;
      }
    }
    Sample s;
    if (!sample(s) || !conditionsOk(s, start, pwmCap) || s.torque != 1) {
      Serial.println("SERVO TEST ABORT: telemetry/read/safety check failed.");
      return false;
    }
    Serial.printf("%s %s t=%lu pos=%ld current_mA=%ld pwm=%ld voltage=%.1f temp=%ld torque=%ld\n",
                  COMMAND, stage, static_cast<unsigned long>(millis() - began),
                  static_cast<long>(s.position), static_cast<long>(s.current),
                  static_cast<long>(s.pwm), s.voltage * 0.1,
                  static_cast<long>(s.temperature), static_cast<long>(s.torque));
    delay(40);
  }
  return true;
}

void run() {
  int32_t value;
  for (uint8_t id = 11; id <= 18; ++id) {
    if (!read(TORQUE_ENABLE, value, id) || value != 0) {
      Serial.printf("%s REFUSED: all eight servos must respond with torque off.\n", COMMAND);
      return;
    }
  }
  if (!read(OPERATING_MODE, value) || value != 4 ||
      !read(DRIVE_MODE, value) || value != (ID == 18 ? 5 : 4) ||
      !read(HOMING_OFFSET, value) || value != (ID == 18 ? -4096 : 4096) ||
      !read(BUS_WATCHDOG, value) || value != 0) {
    Serial.printf("%s REFUSED: unexpected mode/calibration/watchdog.\n", COMMAND);
    return;
  }
  Sample initial;
  if (!sample(initial) || !conditionsOk(initial, initial.position, MAX_PWM) ||
      initial.torque != 0 || initial.position < 4400 || initial.position > 6250) {
    Serial.printf("%s REFUSED: unsafe/unknown initial state or position.\n", COMMAND);
    return;
  }
  int32_t oldPwm, oldDuration, oldAcceleration, pwmLimit;
  if (!read(GOAL_PWM, oldPwm) || !read(PROFILE_VELOCITY, oldDuration) ||
      !read(PROFILE_ACCELERATION, oldAcceleration) || !read(PWM_LIMIT, pwmLimit) ||
      oldPwm <= 0 || pwmLimit <= 0) {
    Serial.printf("%s REFUSED: cannot save output/profile settings.\n", COMMAND);
    return;
  }
  int32_t cap = oldPwm < MAX_PWM ? oldPwm : MAX_PWM;
  if (pwmLimit < cap) cap = pwmLimit;
  Serial.printf("%s START start=%ld target=%ld pwm_cap=%ld\n",
                COMMAND, static_cast<long>(initial.position),
                static_cast<long>(initial.position - STEP_TICKS), static_cast<long>(cap));

  // All changes are RAM-only; every successful setup write is read back before
  // torque can be enabled. A failure at any stage falls through to torque-off.
  bool ok = write(GOAL_PWM, cap) && write(PROFILE_VELOCITY, 1000) &&
            write(PROFILE_ACCELERATION, 250) &&
            write(GOAL_POSITION, initial.position) && write(TORQUE_ENABLE, 1);
  if (ok) ok = write(GOAL_POSITION, initial.position - STEP_TICKS) &&
               observe(initial.position, cap, "out");
  if (ok) ok = write(GOAL_POSITION, initial.position) &&
               observe(initial.position, cap, "return");
  bool off = torqueOff();
  if (off) {
    // Restore profiles/output only after torque-off is verified. Failure to
    // restore is reported, not hidden by a successful movement result.
    bool restored = write(PROFILE_VELOCITY, oldDuration);
    restored = write(PROFILE_ACCELERATION, oldAcceleration) && restored;
    restored = write(GOAL_PWM, oldPwm) && restored;
    Serial.println(restored ? "Servo settings restored; torque OFF." :
                             "Servo torque OFF; settings restoration incomplete. Power-cycle before normal operation.");
    ok = ok && restored;
  }
  Serial.printf("%s %s\n", COMMAND,
                ok && off ? "FINISHED (inspect tracking; this is not a loaded-strength test)." : "ABORTED.");
}
}  // namespace Id18Test

void pollId18TestCommand() {
  static char command[16];
  static uint8_t used = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      command[used] = '\0';
      bool execute = !overflow && strcmp(command, Id18Test::COMMAND) == 0;
      used = 0;
      overflow = false;
      if (execute) {
        Id18Test::run();
        // Never execute commands queued while the previous test was running.
        while (Serial.available()) Serial.read();
        return;
      }
    } else if (used < sizeof(command) - 1 && !overflow) {
      command[used++] = c;
    } else {
      overflow = true;
    }
  }
}
