#pragma once

namespace Alignment18 {
constexpr int32_t TARGET = 5622;
constexpr int32_t MAX_PWM = 266; // About 30% duty, for the detached linkage only.
constexpr uint32_t MOVE_MS = 4000;
constexpr uint32_t HOLD_MS = 300000;
bool active = false, holding = false;
int32_t startPosition = 0, oldPwm = 0, oldDuration = 0, oldAcceleration = 0, cap = 0;
uint32_t began = 0, holdBegan = 0, lastPoll = 0, lastLog = 0, settledSince = 0;

bool read(uint8_t item, int32_t& value) { return Replacement18::read(item, 18, value); }

bool ramWrite(uint8_t item, int32_t value) {
  if (item != TORQUE_ENABLE && item != GOAL_PWM && item != PROFILE_VELOCITY &&
      item != PROFILE_ACCELERATION && item != GOAL_POSITION) return false;
  if (item == TORQUE_ENABLE && value != 0 && value != 1) return false;
  if (item == GOAL_POSITION && (value < 4400 || value > 6250)) return false;
  if (item == GOAL_PWM && value > cap && !Replacement18::matches(TORQUE_ENABLE, 18, 0)) return false;
  if (!dxl.writeControlTableItem(item, uint8_t{18}, value, uint32_t{30})) return false;
  return Replacement18::matches(item, 18, value);
}

bool torqueOff() {
  for (int i = 0; i < 3; ++i) {
    dxl.writeControlTableItem(static_cast<uint8_t>(TORQUE_ENABLE), uint8_t{18}, int32_t{0}, uint32_t{30});
    int32_t torque = dxl.readControlTableItem(static_cast<uint8_t>(TORQUE_ENABLE), uint8_t{18}, uint32_t{30});
    if (dxl.getLastLibErrCode() == 0 &&
        (dxl.getLastStatusPacketError() & 0x7f) == 0 && torque == 0) return true;
  }
  Serial.println("ALIGN18 TORQUE-OFF NOT CONFIRMED: SWITCH BATTERY POWER OFF.");
  return false;
}

void release() {
  bool wasActive = active;
  active = false; holding = false;
  if (!torqueOff()) return;
  bool restored = true;
  if (wasActive) {
    restored = ramWrite(PROFILE_VELOCITY, oldDuration);
    restored = ramWrite(PROFILE_ACCELERATION, oldAcceleration) && restored;
    restored = ramWrite(GOAL_PWM, oldPwm) && restored;
  }
  Serial.println(restored ? "ALIGN18 RELEASED: torque OFF; settings restored." :
                           "ALIGN18 RELEASED: torque OFF; restoration incomplete. Power-cycle before normal operation.");
}

void begin() {
  if (active) { Serial.println("ALIGN18 already active; no new movement started."); return; }
  int32_t existing[7][sizeof(Replacement18::CHECK_ITEMS)];
  int32_t voltage, temperature, pwmLimit;
  if (!Replacement18::verifyTarget() || !Replacement18::existingSnapshot(existing) ||
      !read(PRESENT_POSITION, startPosition) || startPosition < 4400 || startPosition > 6250 ||
      !read(PRESENT_INPUT_VOLTAGE, voltage) || voltage < 35 || voltage > 55 ||
      !read(PRESENT_TEMPERATURE, temperature) || temperature >= 45 ||
      !read(GOAL_PWM, oldPwm) || oldPwm <= 0 || !read(PWM_LIMIT, pwmLimit) || pwmLimit <= 0 ||
      !read(PROFILE_VELOCITY, oldDuration) || !read(PROFILE_ACCELERATION, oldAcceleration) ||
      !Replacement18::matches(BUS_WATCHDOG, 18, 0)) {
    Serial.println("ALIGN18 REFUSED: configuration, torque, position or health check failed.");
    return;
  }
  cap = oldPwm < MAX_PWM ? oldPwm : MAX_PWM;
  if (pwmLimit < cap) cap = pwmLimit;
  active = true; holding = false; settledSince = 0;
  bool ok = ramWrite(GOAL_PWM, cap) && ramWrite(PROFILE_VELOCITY, MOVE_MS) &&
            ramWrite(PROFILE_ACCELERATION, 1000) && ramWrite(GOAL_POSITION, startPosition) &&
            ramWrite(TORQUE_ENABLE, 1) && ramWrite(GOAL_POSITION, TARGET);
  if (!ok) { Serial.println("ALIGN18 ABORT: setup write/readback failed."); release(); return; }
  began = millis(); lastPoll = began - 50; lastLog = began - 250;
  Serial.printf("ALIGN18 MOVING: %ld -> %ld ticks over 4 seconds, PWM cap %ld. Keep hands clear.\n",
                static_cast<long>(startPosition), static_cast<long>(TARGET), static_cast<long>(cap));
}

void tick() {
  if (!active || millis() - lastPoll < 50) return;
  lastPoll = millis();
  int32_t position, current, pwm, voltage, temperature, torque, fault;
  bool ok = read(HARDWARE_ERROR_STATUS, fault) && fault == 0 &&
            read(PRESENT_POSITION, position) && read(PRESENT_CURRENT, current) &&
            read(PRESENT_PWM, pwm) && read(PRESENT_INPUT_VOLTAGE, voltage) &&
            read(PRESENT_TEMPERATURE, temperature) && read(TORQUE_ENABLE, torque);
  int32_t low = (startPosition < TARGET ? startPosition : TARGET) - 64;
  int32_t high = (startPosition > TARGET ? startPosition : TARGET) + 64;
  if (!ok || position < low || position > high || current < -250 || current > 250 ||
      pwm < -cap || pwm > cap || voltage < 35 || voltage > 55 || temperature >= 45 || torque != 1) {
    Serial.println("ALIGN18 ABORT: telemetry/read/safety check failed."); release(); return;
  }
  uint32_t now = millis();
  int32_t error = position > TARGET ? position - TARGET : TARGET - position;
  if (now - lastLog >= (holding ? 1000u : 250u)) {
    lastLog = now;
    Serial.printf("ALIGN18 %s pos=%ld current_mA=%ld pwm=%ld voltage=%.1f temp=%ld\n",
                  holding ? "holding" : "moving", static_cast<long>(position),
                  static_cast<long>(current), static_cast<long>(pwm), voltage * 0.1,
                  static_cast<long>(temperature));
  }
  if (!holding) {
    if (now - began >= MOVE_MS && error <= 6) {
      if (settledSince == 0) settledSince = now;
      if (now - settledSince >= 200) {
        holding = true; holdBegan = now;
        Serial.println("ALIGN18 READY: mounting position reached; holding torque for at most 5 minutes. RELEASE18 or ! releases.");
      }
    } else settledSince = 0;
    if (!holding && now - began >= MOVE_MS + 3000) {
      Serial.println("ALIGN18 ABORT: mounting position not reached in time."); release();
    }
  } else if (error > 32 || now - holdBegan >= HOLD_MS) {
    Serial.println("ALIGN18 STOP: holding time elapsed or mounting position disturbed."); release();
  }
}
} // namespace Alignment18
