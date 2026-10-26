#pragma once
#include <stdint.h>
#include <string.h>

// Head-servo configuration, modelled on Replacement18. It configures ID 19
// only, never writes to IDs 11-18, never enables torque and never commands a
// movement. The last step points Goal Position at the head's current Present
// Position, so the first time the robot firmware broadcasts torque-on the head
// holds still instead of snapping to whatever its goal register happened to
// contain.
namespace HeadConfig {
constexpr uint8_t TARGET = 19;
constexpr int32_t MODEL = 1190; // ROBOTIS XL330-M077-T.
constexpr uint32_t FINAL_BAUD = 1000000;
constexpr uint32_t FACTORY_BAUD = 57600;
constexpr int32_t DRIVE = 4;        // Time-based profile, normal direction.
constexpr int32_t OPERATING = 4;    // Extended position control.
constexpr int32_t OFFSET = 2048;    // Mechanical centre reads 0 deg (4096).
constexpr uint8_t FIRST_LEG = 11;
constexpr uint8_t LEG_COUNT = 8;    // IDs 11-18 must all be present.
constexpr uint8_t CHECK_ITEMS[] = {ID, BAUD_RATE, DRIVE_MODE, OPERATING_MODE,
                                   HOMING_OFFSET, TORQUE_ENABLE};
struct Candidate { uint8_t id; uint32_t baud; };

bool read(uint8_t item, uint8_t id, int32_t& value) {
  value = dxl.readControlTableItem(item, id, 30);
  return dxl.getLastLibErrCode() == 0 && dxl.getLastStatusPacketError() == 0;
}

bool matches(uint8_t item, uint8_t id, int32_t expected) {
  int32_t value;
  return read(item, id, value) && value == expected;
}

bool put(uint8_t item, uint8_t id, int32_t value) {
  // No path here to enable torque, move a joint, or touch IDs 11-18.
  if ((id != 1 && id != TARGET) ||
      !((item == TORQUE_ENABLE && value == 0) ||
        (item == OPERATING_MODE && value == OPERATING) ||
        (item == DRIVE_MODE && value == DRIVE) ||
        (item == HOMING_OFFSET && value == OFFSET))) return false;
  int32_t before;
  if (!read(item, id, before)) return false;
  if (before == value) return true;
  bool ack = dxl.writeControlTableItem(item, id, value, 30);
  delay(100);
  return ack && matches(item, id, value);
}

uint8_t findCandidate(Candidate& candidate) {
  uint8_t count = 0;
  const uint32_t bauds[] = {FINAL_BAUD, FACTORY_BAUD};
  const uint8_t ids[] = {1, TARGET};
  for (auto baud : bauds) {
    dxl.begin(baud);
    for (auto id : ids) {
      if (dxl.ping(id)) {
        candidate = {id, baud};
        ++count;
      }
    }
  }
  dxl.begin(FINAL_BAUD);
  return count;
}

bool legSnapshot(int32_t values[LEG_COUNT][sizeof(CHECK_ITEMS)]) {
  dxl.begin(FINAL_BAUD);
  for (uint8_t i = 0; i < LEG_COUNT; ++i) {
    for (uint8_t j = 0; j < sizeof(CHECK_ITEMS); ++j) {
      if (!read(CHECK_ITEMS[j], FIRST_LEG + i, values[i][j])) return false;
    }
    // Right ID, 1 Mbps, torque off.
    if (values[i][0] != FIRST_LEG + i || values[i][1] != 3 || values[i][5] != 0) return false;
  }
  return true;
}

bool verifyTarget() {
  dxl.begin(FINAL_BAUD);
  return matches(MODEL_NUMBER, TARGET, MODEL) && matches(ID, TARGET, TARGET) &&
         matches(BAUD_RATE, TARGET, 3) && matches(OPERATING_MODE, TARGET, OPERATING) &&
         matches(DRIVE_MODE, TARGET, DRIVE) && matches(HOMING_OFFSET, TARGET, OFFSET) &&
         matches(TORQUE_ENABLE, TARGET, 0) && matches(HARDWARE_ERROR_STATUS, TARGET, 0);
}

// Park the goal where the servo already is, so torque-on causes no movement.
bool parkGoalAtPresent() {
  int32_t present;
  if (!read(PRESENT_POSITION, TARGET, present)) return false;
  if (matches(GOAL_POSITION, TARGET, present)) return true;
  bool ack = dxl.writeControlTableItem(static_cast<uint8_t>(GOAL_POSITION), TARGET,
                                       present, uint32_t{30});
  delay(100);
  return ack && matches(GOAL_POSITION, TARGET, present);
}

bool configure() {
  int32_t before[LEG_COUNT][sizeof(CHECK_ITEMS)];
  if (!legSnapshot(before)) {
    Serial.println("CONFIGHEAD REFUSED: IDs 11-18 must respond at 1 Mbps with torque off.");
    return false;
  }
  Candidate candidate{};
  uint8_t found = findCandidate(candidate);
  if (found != 1) {
    Serial.printf("CONFIGHEAD REFUSED: expected one head servo at ID1/ID19, found %u.\n", found);
    return false;
  }
  dxl.begin(candidate.baud);
  int32_t voltage, temperature;
  if (!matches(MODEL_NUMBER, candidate.id, MODEL) ||
      !matches(HARDWARE_ERROR_STATUS, candidate.id, 0) ||
      !read(PRESENT_INPUT_VOLTAGE, candidate.id, voltage) || voltage < 35 || voltage > 55 ||
      !read(PRESENT_TEMPERATURE, candidate.id, temperature) || temperature >= 45) {
    Serial.println("CONFIGHEAD REFUSED: model, communication, or health check failed.");
    return false;
  }
  if (!put(TORQUE_ENABLE, candidate.id, 0) ||
      !put(OPERATING_MODE, candidate.id, OPERATING) ||
      !put(DRIVE_MODE, candidate.id, DRIVE) ||
      !put(HOMING_OFFSET, candidate.id, OFFSET)) {
    Serial.println("CONFIGHEAD STOPPED: setting readback failed. Torque was not enabled.");
    return false;
  }

  // A reply can be lost when an address or baud changes. Verification happens
  // at the new address/speed, and repeating the command recovers either state.
  if (candidate.id != TARGET) {
    dxl.writeControlTableItem(static_cast<uint8_t>(ID), candidate.id, int32_t{TARGET}, uint32_t{30});
    delay(100);
    if (!matches(ID, TARGET, TARGET)) {
      Serial.println("CONFIGHEAD STOPPED: new ID not confirmed; retry after checking STATUS.");
      return false;
    }
  }
  if (candidate.baud != FINAL_BAUD) {
    dxl.writeControlTableItem(static_cast<uint8_t>(BAUD_RATE), TARGET, int32_t{3}, uint32_t{30});
    delay(100);
  }
  if (!verifyTarget()) {
    Serial.println("CONFIGHEAD STOPPED: final settings not confirmed; check STATUS.");
    return false;
  }
  if (!parkGoalAtPresent()) {
    Serial.println("CONFIGHEAD STOPPED: could not park the goal at the present position.");
    Serial.println("Do not power the robot firmware until this is confirmed.");
    return false;
  }
  int32_t after[LEG_COUNT][sizeof(CHECK_ITEMS)];
  if (!legSnapshot(after) || memcmp(before, after, sizeof(before)) != 0) {
    Serial.println("CONFIGHEAD WARNING: head verified, but IDs 11-18 could not be confirmed unchanged.");
    return false;
  }
  int32_t present = 0;
  read(PRESENT_POSITION, TARGET, present);
  Serial.println("CONFIGHEAD SUCCESS: ID19, 1 Mbps, mode4, drive4, offset+2048, torque OFF.");
  Serial.printf("  Present position %ld ticks (%.1f deg from centre). Goal parked here.\n",
                static_cast<long>(present), (present - 4096) * 360.0 / 4096.0);
  Serial.println("  IDs 11-18 settings unchanged. Recheck the head goal after hand adjustment or power cycling.");
  return true;
}

void status() {
  Candidate candidate{};
  uint8_t found = findCandidate(candidate);
  Serial.printf("HEAD STATUS: %u candidate(s) at ID1/ID19 and 57600/1000000 baud.\n", found);
  if (found == 1) {
    dxl.begin(candidate.baud);
    Serial.printf("  Candidate ID%u at %lu baud\n", candidate.id,
                  static_cast<unsigned long>(candidate.baud));
    const uint8_t items[] = {MODEL_NUMBER, OPERATING_MODE, DRIVE_MODE, HOMING_OFFSET,
                             TORQUE_ENABLE, PRESENT_POSITION, GOAL_POSITION,
                             HARDWARE_ERROR_STATUS, PRESENT_INPUT_VOLTAGE, PRESENT_TEMPERATURE};
    const char* names[] = {"Model", "Operating mode", "Drive mode", "Homing offset",
                           "Torque", "Position ticks", "Goal ticks", "Hardware errors",
                           "Voltage (0.1 V)", "Temperature (C)"};
    for (uint8_t i = 0; i < sizeof(items); ++i) {
      int32_t value;
      if (read(items[i], candidate.id, value)) Serial.printf("  %s: %ld\n", names[i], static_cast<long>(value));
      else Serial.printf("  %s: READ FAILED\n", names[i]);
    }
  }
  dxl.begin(FINAL_BAUD);
  uint8_t legs = 0;
  for (uint8_t i = 0; i < LEG_COUNT; ++i) if (dxl.ping(FIRST_LEG + i)) ++legs;
  Serial.printf("  Leg motors answering at 1 Mbps: %u of %u\n", legs, LEG_COUNT);
}

// Read-only bench inventory. With torque off, compare before/after positions
// while gently turning one detached output to identify its physical location.
void legStatus() {
  dxl.begin(FINAL_BAUD);
  Serial.println("LEG STATUS (read-only): positions do not establish the physical crank mapping.");
  const uint8_t items[] = {MODEL_NUMBER, ID, BAUD_RATE, OPERATING_MODE, DRIVE_MODE,
                           HOMING_OFFSET, TORQUE_ENABLE, PRESENT_POSITION,
                           HARDWARE_ERROR_STATUS, PRESENT_INPUT_VOLTAGE, PRESENT_TEMPERATURE};
  const char* names[] = {"model", "id", "baud_reg", "mode", "drive", "offset",
                         "torque", "position", "errors", "voltage_0.1V", "temp_C"};
  for (uint8_t id = FIRST_LEG; id < FIRST_LEG + LEG_COUNT; ++id) {
    Serial.printf("ID%u", id);
    for (uint8_t j = 0; j < sizeof(items); ++j) {
      int32_t value;
      if (read(items[j], id, value)) Serial.printf(" %s=%ld", names[j], static_cast<long>(value));
      else Serial.printf(" %s=READ_FAILED", names[j]);
    }
    Serial.println("");
  }
}
} // namespace HeadConfig
