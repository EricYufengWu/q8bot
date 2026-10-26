#pragma once

// Commanded bench alignment, one detached leg pair or the head at a time.
// No EEPROM writes, broadcast torque or wireless input.
namespace SquroAssembly {
constexpr uint32_t MOVE_MS = 5000, HOLD_MS = 300000, KEEP_MS = 10000;
constexpr int32_t FRONT_TARGETS[] = {4695, 5564};
// Rear CAD angles: elbow -41.286 deg, shoulder +57.000 deg, in the same
// nose/down frame. Match the existing robot's int(deg*4096/360+0.5)+4096.
constexpr int32_t REAR_TARGETS[] = {3627, 4745};
constexpr int32_t OFFSETS[] = {2048, 4096, -2048, -4096, 2048, 4096, -2048, -4096, 2048};
constexpr int32_t DRIVES[] = {4, 4, 5, 5, 4, 4, 5, 5, 4};
struct Motor {
  uint8_t id = 0;
  int32_t target = 0, start = 0, cap = 0, oldPwm = 0, oldDuration = 0, oldAccel = 0;
};
Motor motors[2];
uint8_t count = 0;
bool active = false, holding = false, pendingRestore = false, blocked = false;
uint32_t began = 0, holdBegan = 0, lastKeep = 0, lastPoll = 0, lastLog = 0, settledSince = 0;

bool read(uint8_t item, uint8_t id, int32_t& value) { return HeadConfig::read(item, id, value); }
bool matches(uint8_t item, uint8_t id, int32_t value) { return HeadConfig::matches(item, id, value); }

bool torqueOff(uint8_t id) {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    dxl.writeControlTableItem(static_cast<uint8_t>(TORQUE_ENABLE), id, int32_t{0}, uint32_t{30});
    int32_t value = dxl.readControlTableItem(static_cast<uint8_t>(TORQUE_ENABLE), id, uint32_t{30});
    // A latched hardware alert must not hide successful torque-off.
    if (dxl.getLastLibErrCode() == 0 && (dxl.getLastStatusPacketError() & 0x7f) == 0 && value == 0) return true;
  }
  Serial.printf("ASSEMBLY ID%u TORQUE-OFF NOT CONFIRMED: SWITCH BATTERY POWER OFF.\n", id);
  return false;
}

// At MCU restart, release any powered servo left holding by the previous run.
void startupRelease() {
  dxl.begin(HeadConfig::FINAL_BAUD);
  for (uint8_t id = 11; id <= 19; ++id) {
    if (dxl.ping(id) && !torqueOff(id)) blocked = true;
  }
}

bool ramWrite(Motor& m, uint8_t item, int32_t value) {
  if (m.id < 11 || m.id > 19) return false;
  if (item != TORQUE_ENABLE && item != GOAL_PWM && item != PROFILE_VELOCITY &&
      item != PROFILE_ACCELERATION && item != GOAL_POSITION) return false;
  if (item == TORQUE_ENABLE && value != 0 && value != 1) return false;
  if (item == GOAL_POSITION && value != m.start && value != m.target) return false;
  if (item == GOAL_PWM && (value < 0 ||
      (value > m.cap && !matches(TORQUE_ENABLE, m.id, 0)))) return false;
  if (!dxl.writeControlTableItem(item, m.id, value, uint32_t{30})) return false;
  return matches(item, m.id, value);
}

void release() {
  active = false; holding = false;
  bool off = true;
  if (count) {
    // Attempt both even if the first fails. Restore nothing until both are off.
    for (uint8_t i = 0; i < count; ++i) off = torqueOff(motors[i].id) && off;
  } else {
    for (uint8_t id = 11; id <= 19; ++id) off = torqueOff(id) && off;
  }
  if (!off) { blocked = true; return; }
  bool restored = true;
  if (pendingRestore) {
    for (uint8_t i = 0; i < count; ++i) {
      Motor& m = motors[i];
      restored = ramWrite(m, PROFILE_VELOCITY, m.oldDuration) && restored;
      restored = ramWrite(m, PROFILE_ACCELERATION, m.oldAccel) && restored;
      restored = ramWrite(m, GOAL_PWM, m.oldPwm) && restored;
    }
  }
  blocked = !restored;
  if (restored) { pendingRestore = false; count = 0; }
  Serial.println(restored ? "ASSEMBLY RELEASED: torque OFF; profiles/output restored." :
                           "ASSEMBLY RELEASED: torque OFF; restoration failed. Retry RELEASE before proceeding.");
}

bool preflight() {
  dxl.begin(HeadConfig::FINAL_BAUD);
  for (uint8_t id = 11; id <= 19; ++id) {
    int32_t voltage, temperature;
    if (!matches(MODEL_NUMBER, id, HeadConfig::MODEL) || !matches(ID, id, id) ||
        !matches(BAUD_RATE, id, 3) || !matches(OPERATING_MODE, id, 4) ||
        !matches(DRIVE_MODE, id, DRIVES[id - 11]) ||
        !matches(HOMING_OFFSET, id, OFFSETS[id - 11]) ||
        !matches(TORQUE_ENABLE, id, 0) || !matches(HARDWARE_ERROR_STATUS, id, 0) ||
        !matches(BUS_WATCHDOG, id, 0) || !read(PRESENT_INPUT_VOLTAGE, id, voltage) ||
        voltage < 35 || voltage > 55 || !read(PRESENT_TEMPERATURE, id, temperature) || temperature >= 45) {
      Serial.printf("ASSEMBLY REFUSED: ID%u configuration, torque, communication or health check failed.\n", id);
      return false;
    }
  }
  return true;
}

void begin(uint8_t firstId) {
  if (active || pendingRestore || blocked) {
    Serial.println("ASSEMBLY REFUSED: RELEASE the current alignment first."); return;
  }
  if (firstId != 11 && firstId != 13 && firstId != 15 && firstId != 17 && firstId != 19) return;
  if (!preflight()) return;
  count = firstId == 19 ? 1 : 2;
  for (uint8_t i = 0; i < count; ++i) {
    Motor& m = motors[i];
    m = Motor{}; m.id = firstId + i;
    bool rear = firstId == 15 || firstId == 17;
    m.target = firstId == 19 ? 4096 : (rear ? REAR_TARGETS[i] : FRONT_TARGETS[i]);
    int32_t pwmLimit;
    // The detached rear outputs can start nearly half a turn from their CAD
    // reference (ID16 measured 6688 ticks). Refuse more than one half-turn.
    const int32_t travel = firstId == 19 ? 256 : (rear ? 2048 : 1536);
    if (!read(PRESENT_POSITION, m.id, m.start) ||
        m.start < m.target - travel || m.start > m.target + travel ||
        !read(GOAL_PWM, m.id, m.oldPwm) || m.oldPwm <= 0 ||
        !read(PWM_LIMIT, m.id, pwmLimit) || pwmLimit <= 0 ||
        !read(PROFILE_VELOCITY, m.id, m.oldDuration) ||
        !read(PROFILE_ACCELERATION, m.id, m.oldAccel)) {
      Serial.printf("ASSEMBLY REFUSED: ID%u position or RAM settings outside checked range.\n", m.id);
      count = 0; return;
    }
    m.cap = firstId == 19 ? 133 : 266;
    if (m.oldPwm < m.cap) m.cap = m.oldPwm;
    if (pwmLimit < m.cap) m.cap = pwmLimit;
  }
  // Every setting above was captured before any RAM write or torque-on.
  pendingRestore = true;
  active = true; holding = false; settledSince = 0;
  for (uint8_t i = 0; i < count; ++i) {
    Motor& m = motors[i];
    if (!ramWrite(m, GOAL_PWM, m.cap) || !ramWrite(m, PROFILE_VELOCITY, MOVE_MS) ||
        !ramWrite(m, PROFILE_ACCELERATION, MOVE_MS / 4) || !ramWrite(m, GOAL_POSITION, m.start)) {
      Serial.println("ASSEMBLY ABORT: preparation write/readback failed."); release(); return;
    }
  }
  for (uint8_t i = 0; i < count; ++i) {
    Motor& m = motors[i];
    if (!ramWrite(m, TORQUE_ENABLE, 1) || !ramWrite(m, GOAL_POSITION, m.target)) {
      Serial.println("ASSEMBLY ABORT: torque/goal write/readback failed."); release(); return;
    }
    Serial.printf("ASSEMBLY MOVING ID%u: %ld -> %ld ticks over 5 seconds, PWM cap %ld.\n",
                  m.id, static_cast<long>(m.start), static_cast<long>(m.target), static_cast<long>(m.cap));
  }
  began = lastKeep = millis(); lastPoll = began - 50; lastLog = began - 250;
}

void tick() {
  if (!active) return;
  uint32_t now = millis();
  if (now - lastKeep >= KEEP_MS || (holding && now - holdBegan >= HOLD_MS)) {
    Serial.println("ASSEMBLY STOP: host heartbeat lost or five-minute hold expired."); release(); return;
  }
  if (now - lastPoll < 50) return;
  lastPoll = now;
  bool settled = true;
  bool log = now - lastLog >= (holding ? 1000u : 250u);
  for (uint8_t i = 0; i < count; ++i) {
    Motor& m = motors[i];
    int32_t position = 0, current = 0, pwm = 0, voltage = 0, temperature = 0, torque = 0, fault = 0;
    bool ok = read(HARDWARE_ERROR_STATUS, m.id, fault) && fault == 0 &&
              read(PRESENT_POSITION, m.id, position) && read(PRESENT_CURRENT, m.id, current) &&
              read(PRESENT_PWM, m.id, pwm) && read(PRESENT_INPUT_VOLTAGE, m.id, voltage) &&
              read(PRESENT_TEMPERATURE, m.id, temperature) && read(TORQUE_ENABLE, m.id, torque);
    int32_t low = (m.start < m.target ? m.start : m.target) - 64;
    int32_t high = (m.start > m.target ? m.start : m.target) + 64;
    int32_t maxCurrent = m.id == 19 ? 150 : 250;
    int32_t error = position > m.target ? position - m.target : m.target - position;
    if (!ok || position < low || position > high || current < -maxCurrent || current > maxCurrent ||
        pwm < -m.cap || pwm > m.cap || voltage < 35 || voltage > 55 || temperature >= 45 ||
        torque != 1 || (holding && error > 32)) {
      Serial.printf("ASSEMBLY ABORT: ID%u telemetry/read check failed.\n", m.id);
      release();
      Serial.printf("  read_ok=%u pos=%ld target=%ld current_mA=%ld pwm=%ld voltage_0.1V=%ld temp=%ld torque=%ld fault=%ld\n",
                    ok ? 1u : 0u, static_cast<long>(position), static_cast<long>(m.target),
                    static_cast<long>(current), static_cast<long>(pwm), static_cast<long>(voltage),
                    static_cast<long>(temperature), static_cast<long>(torque), static_cast<long>(fault));
      return;
    }
    if (error > 6) settled = false;
    if (log) Serial.printf("ASSEMBLY %s ID%u pos=%ld current_mA=%ld pwm=%ld voltage=%.1f temp=%ld\n",
                          holding ? "holding" : "moving", m.id, static_cast<long>(position),
                          static_cast<long>(current), static_cast<long>(pwm), voltage * 0.1,
                          static_cast<long>(temperature));
  }
  if (log) lastLog = now;
  if (!holding) {
    if (now - began >= MOVE_MS && settled) {
      if (settledSince == 0) settledSince = now;
      if (now - settledSince >= 200) {
        holding = true; holdBegan = now;
        Serial.println("ASSEMBLY READY: measured targets reached; holding up to 5 minutes. RELEASE or ! stops.");
      }
    } else settledSince = 0;
    if (!holding && now - began >= MOVE_MS + 3000) {
      Serial.println("ASSEMBLY ABORT: targets not reached in time."); release();
    }
  }
}

// Used by both the Arduino loop and host tests, including overflow handling.
void consume(char c) {
  static char command[24];
  static uint8_t used = 0;
  static bool overflow = false;
  if (c == '!') { release(); used = 0; overflow = false; return; }
  if (c == '\r') return;
  if (c != '\n') {
    if (!overflow && used < sizeof(command) - 1) command[used++] = c;
    else overflow = true;
    return;
  }
  command[used] = '\0';
  if (!overflow && strcmp(command, "RELEASE") == 0) release();
  else if (!overflow && strcmp(command, "KEEP") == 0) lastKeep = millis();
  else if (active) Serial.println("ASSEMBLY active: KEEP, RELEASE or ! only.");
  else if (!overflow && strcmp(command, "CENTERHEAD") == 0) begin(19);
  else if (!overflow && strcmp(command, "ALIGNFL") == 0) begin(11);
  else if (!overflow && strcmp(command, "ALIGNFR") == 0) begin(13);
  else if (!overflow && strcmp(command, "ALIGNBL") == 0) begin(15);
  else if (!overflow && strcmp(command, "ALIGNBR") == 0) begin(17);
  else if (!overflow && strcmp(command, "STATUS") == 0) HeadConfig::status();
  else if (!overflow && strcmp(command, "LEGS") == 0) HeadConfig::legStatus();
  else if (used) Serial.println("Commands: CENTERHEAD, ALIGNFL, ALIGNFR, ALIGNBL, ALIGNBR, STATUS, LEGS, KEEP, RELEASE, !");
  used = 0; overflow = false;
}
} // namespace SquroAssembly
