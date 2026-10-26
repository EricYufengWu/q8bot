#pragma once
#include <stdint.h>
#include <string.h>

namespace Replacement18 {
constexpr uint8_t TARGET = 18;
constexpr int32_t MODEL = 1190; // ROBOTIS XL330-M077-T.
constexpr uint32_t FINAL_BAUD = 1000000;
constexpr uint32_t FACTORY_BAUD = 57600;
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
  // This helper has no path to enable torque, move a joint, or write IDs 11–17.
  if ((id != 1 && id != TARGET) ||
      !((item == TORQUE_ENABLE && value == 0) ||
        (item == OPERATING_MODE && value == 4) ||
        (item == DRIVE_MODE && value == 5) ||
        (item == HOMING_OFFSET && value == -4096))) return false;
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

bool existingSnapshot(int32_t values[7][sizeof(CHECK_ITEMS)]) {
  dxl.begin(FINAL_BAUD);
  for (uint8_t i = 0; i < 7; ++i) {
    for (uint8_t j = 0; j < sizeof(CHECK_ITEMS); ++j) {
      if (!read(CHECK_ITEMS[j], 11 + i, values[i][j])) return false;
    }
    if (values[i][0] != 11 + i || values[i][1] != 3 || values[i][5] != 0) return false;
  }
  return true;
}

bool verifyTarget() {
  dxl.begin(FINAL_BAUD);
  return matches(MODEL_NUMBER, TARGET, MODEL) && matches(ID, TARGET, TARGET) &&
         matches(BAUD_RATE, TARGET, 3) && matches(OPERATING_MODE, TARGET, 4) &&
         matches(DRIVE_MODE, TARGET, 5) && matches(HOMING_OFFSET, TARGET, -4096) &&
         matches(TORQUE_ENABLE, TARGET, 0) && matches(HARDWARE_ERROR_STATUS, TARGET, 0);
}

bool configure() {
  int32_t before[7][sizeof(CHECK_ITEMS)];
  if (!existingSnapshot(before)) {
    Serial.println("CONFIG18 REFUSED: IDs 11–17 must respond at 1 Mbps with torque off.");
    return false;
  }
  Candidate candidate{};
  uint8_t found = findCandidate(candidate);
  if (found != 1) {
    Serial.printf("CONFIG18 REFUSED: expected one replacement at ID1/ID18, found %u candidates.\n", found);
    return false;
  }
  dxl.begin(candidate.baud);
  int32_t voltage, temperature;
  if (!matches(MODEL_NUMBER, candidate.id, MODEL) ||
      !matches(HARDWARE_ERROR_STATUS, candidate.id, 0) ||
      !read(PRESENT_INPUT_VOLTAGE, candidate.id, voltage) || voltage < 35 || voltage > 55 ||
      !read(PRESENT_TEMPERATURE, candidate.id, temperature) || temperature >= 45) {
    Serial.println("CONFIG18 REFUSED: model, communication, or health check failed.");
    return false;
  }
  if (!put(TORQUE_ENABLE, candidate.id, 0) ||
      !put(OPERATING_MODE, candidate.id, 4) ||
      !put(DRIVE_MODE, candidate.id, 5) ||
      !put(HOMING_OFFSET, candidate.id, -4096)) {
    Serial.println("CONFIG18 STOPPED: setting readback failed. Torque was not enabled.");
    return false;
  }

  // A reply can be lost when an address/baud changes. Verification happens at
  // the new address/speed, and a later explicit retry can recover either state.
  if (candidate.id != TARGET) {
    dxl.writeControlTableItem(static_cast<uint8_t>(ID), candidate.id, int32_t{TARGET}, uint32_t{30});
    delay(100);
    if (!matches(ID, TARGET, TARGET)) {
      Serial.println("CONFIG18 STOPPED: new ID not confirmed; retry after checking STATUS.");
      return false;
    }
  }
  if (candidate.baud != FINAL_BAUD) {
    dxl.writeControlTableItem(static_cast<uint8_t>(BAUD_RATE), TARGET, int32_t{3}, uint32_t{30});
    delay(100);
  }
  if (!verifyTarget()) {
    Serial.println("CONFIG18 STOPPED: final settings not confirmed; check STATUS.");
    return false;
  }
  int32_t after[7][sizeof(CHECK_ITEMS)];
  if (!existingSnapshot(after) || memcmp(before, after, sizeof(before)) != 0) {
    Serial.println("CONFIG18 WARNING: target verified, but unchanged settings of IDs 11–17 not confirmed.");
    return false;
  }
  Serial.println("CONFIG18 SUCCESS: ID18, 1 Mbps, mode4, drive5, offset-4096, torque OFF.");
  Serial.println("IDs 11–17 settings unchanged. Mounting-position alignment is still required.");
  return true;
}

void status() {
  Candidate candidate{};
  uint8_t found = findCandidate(candidate);
  Serial.printf("REPLACEMENT STATUS: %u candidate(s) at ID1/ID18 and 57600/1000000 baud.\n", found);
  if (found == 1) {
    dxl.begin(candidate.baud);
    Serial.printf("  Candidate ID%u at %lu baud\n", candidate.id, static_cast<unsigned long>(candidate.baud));
    const uint8_t items[] = {MODEL_NUMBER, OPERATING_MODE, DRIVE_MODE, HOMING_OFFSET,
                             TORQUE_ENABLE, PRESENT_POSITION, HARDWARE_ERROR_STATUS};
    const char* names[] = {"Model", "Operating mode", "Drive mode", "Homing offset",
                          "Torque", "Position ticks", "Hardware errors"};
    for (uint8_t i = 0; i < sizeof(items); ++i) {
      int32_t value;
      if (read(items[i], candidate.id, value)) Serial.printf("  %s: %ld\n", names[i], static_cast<long>(value));
      else Serial.printf("  %s: READ FAILED\n", names[i]);
    }
  }
  dxl.begin(FINAL_BAUD);
}
} // namespace Replacement18
