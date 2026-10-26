#define Q8_ALIGNMENT_TEST
#include "test_replacement.cpp"
#include "../src/head_config.h"
#include "../src/squro_assembly.h"

namespace A = SquroAssembly;
void command(const char* s) { while (*s) A::consume(*s++); }
void prepare() {
  reset();
  A::active = A::holding = A::pendingRestore = A::blocked = false;
  A::count = 0;
  dxl.devices.erase(1);
  int positions[] = {5336,5812,3770,5236,3991,6688,4201,5844,3983};
  int offsets[] = {2048,4096,-2048,-4096,2048,4096,-2048,-4096,2048};
  int drives[] = {4,4,5,5,4,4,5,5,4};
  for (uint8_t id = 11; id <= 19; ++id) {
    Device d;
    d.baud = 1000000; d.values[ID] = id; d.values[BAUD_RATE] = 3;
    d.values[OPERATING_MODE] = 4; d.values[DRIVE_MODE] = drives[id - 11];
    d.values[HOMING_OFFSET] = offsets[id - 11];
    d.values[PRESENT_POSITION] = positions[id - 11];
    // Deliberately stale initial goal: must be parked before torque-on.
    d.values[GOAL_POSITION] = 1234;
    dxl.devices[id] = d;
  }
  dxl.writes.clear(); Serial.output.clear();
}
void scope(uint8_t first) {
  for (auto w : dxl.writes) {
    assert(w.id == first || (first != 19 && w.id == first + 1));
    assert(w.item == TORQUE_ENABLE || w.item == GOAL_PWM || w.item == PROFILE_VELOCITY ||
           w.item == PROFILE_ACCELERATION || w.item == GOAL_POSITION);
  }
}
void off(uint8_t first) {
  assert(!A::active && !A::pendingRestore && !A::blocked);
  for (uint8_t id = first; id <= first + (first == 19 ? 0 : 1); ++id) {
    assert(dxl.devices.at(id).values.at(TORQUE_ENABLE) == 0);
    assert(dxl.devices.at(id).values.at(GOAL_PWM) == 885);
    assert(dxl.devices.at(id).values.at(PROFILE_VELOCITY) == 0);
  }
  scope(first);
}
void advance(bool track, int milliseconds) {
  for (int t = 0; t < milliseconds && A::active; t += 50) {
    delay(50); command("KEEP\n");
    if (track) {
      for (uint8_t i = 0; i < A::count; ++i) {
        auto& m = A::motors[i];
        uint32_t elapsed = millis() - A::began;
        dxl.devices[m.id].values[PRESENT_POSITION] = elapsed >= A::MOVE_MS ? m.target :
          m.start + (m.target - m.start) * static_cast<int32_t>(elapsed) / static_cast<int32_t>(A::MOVE_MS);
      }
    }
    A::tick();
  }
}
int main() {
  for (uint8_t first : {11,13,15,17,19}) {
    prepare(); A::begin(first); assert(A::active);
    for (uint8_t i = 0; i < A::count; ++i) {
      auto& m = A::motors[i];
      int expected = first == 19 ? 4096 : (first < 15 ? (i == 0 ? 4695 : 5564) : (i == 0 ? 3627 : 4745));
      assert(dxl.devices[m.id].values[GOAL_POSITION] == expected);
      assert(m.cap == (first == 19 ? 133 : 266));
      bool parked = false, capped = false;
      for (auto w : dxl.writes) if (w.id == m.id) {
        if (w.item == GOAL_POSITION && w.value == m.start) parked = true;
        if (w.item == GOAL_PWM && w.value == m.cap) capped = true;
        if (w.item == TORQUE_ENABLE && w.value == 1) assert(parked && capped);
      }
    }
    advance(true, 5500); assert(A::active && A::holding);
    assert(Serial.output.find("ASSEMBLY READY") != std::string::npos);
    size_t count = dxl.writes.size(); command("ALIGNFL\nALIGNBR\nSTATUS\nLEGS\nCENTERHEAD\n");
    assert(dxl.writes.size() == count);
    command("!ignored\n"); off(first);
  }
  prepare(); command("ALIGNFL\n"); advance(true, 5500);
  delay(A::HOLD_MS); command("KEEP\n"); A::tick(); off(11);
  prepare(); command("CENTERHEAD\n"); delay(A::KEEP_MS); A::tick(); off(19);
  prepare(); command("ALIGNFR\n"); advance(false, 8100); off(13);
  assert(Serial.output.find("targets not reached") != std::string::npos);

  for (auto invalid : std::vector<std::pair<int,int>>{{HARDWARE_ERROR_STATUS,32},
      {PRESENT_CURRENT,251}, {PRESENT_CURRENT,-251}, {PRESENT_PWM,267},
      {PRESENT_INPUT_VOLTAGE,34}, {PRESENT_INPUT_VOLTAGE,56},
      {PRESENT_TEMPERATURE,45}, {PRESENT_POSITION,6000}, {TORQUE_ENABLE,0}}) {
    prepare(); A::begin(11); dxl.devices[12].values[invalid.first] = invalid.second;
    delay(50); A::tick(); off(11);
  }
  prepare(); A::begin(19); dxl.devices[19].values[PRESENT_CURRENT] = 151;
  delay(50); A::tick(); off(19);
  prepare(); A::begin(11); advance(true, 5500);
  dxl.devices[12].values[PRESENT_POSITION] += 33;
  delay(50); A::tick(); off(11);

  for (uint8_t id = 11; id <= 19; ++id) {
    prepare(); dxl.devices.erase(id); A::begin(11);
    assert(!A::active && dxl.writes.empty());
    prepare(); dxl.devices[id].values[TORQUE_ENABLE] = 1; A::begin(11);
    assert(!A::active && dxl.writes.empty());
  }
  for (auto invalid : std::vector<std::pair<int,int>>{{DRIVE_MODE,12}, {OPERATING_MODE,3},
      {HOMING_OFFSET,0}, {BUS_WATCHDOG,-1}, {PRESENT_INPUT_VOLTAGE,34}, {MODEL_NUMBER,1200}}) {
    prepare(); dxl.devices[11].values[invalid.first] = invalid.second;
    A::begin(11); assert(!A::active && dxl.writes.empty());
  }
  prepare(); dxl.devices[19].values[PRESENT_POSITION] = 4353; A::begin(19);
  assert(!A::active && dxl.writes.empty());
  prepare(); dxl.devices[12].values[PRESENT_POSITION] = 10000; A::begin(11);
  assert(!A::active && dxl.writes.empty());
  prepare(); dxl.devices[12].values[GOAL_PWM] = 0; A::begin(11);
  assert(!A::active && dxl.writes.empty());
  prepare(); dxl.devices[11].values[GOAL_PWM] = 120; dxl.devices[12].values[PWM_LIMIT] = 99;
  A::begin(11); assert(A::motors[0].cap == 120 && A::motors[1].cap == 99); A::release();

  for (int item : {GOAL_PWM,PROFILE_VELOCITY,PROFILE_ACCELERATION,GOAL_POSITION,TORQUE_ENABLE}) {
    prepare(); dxl.failedItem = item; A::begin(11);
    assert(!A::active);
    assert(dxl.devices[11].values[TORQUE_ENABLE] == 0 && dxl.devices[12].values[TORQUE_ENABLE] == 0);
    scope(11);
    dxl.failedItem = -1;
    if (A::pendingRestore || A::blocked) A::release();
    off(11);
  }
  // A failed torque-off must not restore high output, nor permit another move.
  prepare(); A::begin(11); dxl.failedItem = TORQUE_ENABLE;
  size_t before = dxl.writes.size(); A::release();
  assert(A::blocked && A::pendingRestore);
  for (size_t i = before; i < dxl.writes.size(); ++i) assert(dxl.writes[i].item == TORQUE_ENABLE);
  before = dxl.writes.size(); A::begin(19); assert(dxl.writes.size() == before);
  dxl.failedItem = -1; A::release(); off(11);

  prepare(); A::begin(11); dxl.devices.erase(12); delay(50); A::tick();
  assert(!A::active && A::blocked && dxl.devices[11].values[TORQUE_ENABLE] == 0);
  assert(dxl.devices[11].values[GOAL_PWM] == 266); // No restoration until both confirmed off.

  prepare(); command("ALIGN11\nALIGNALL\nALIGNFLx\nxxxxxxxxxxxxxxxxxxxxxxxxALIGNFL\n");
  assert(!A::active && dxl.writes.empty());
  command("ALIGNFL\n"); assert(A::active);
  command("xxxxxxxxxxxxxxxxxxxxxxxx!\n"); off(11);
  prepare(); command("ALIGNBL\n"); advance(true, 5500);
  assert(A::holding); command("RELEASE\n"); off(15);
  prepare(); command("ALIGNBR\n"); advance(true, 5500);
  assert(A::holding); command("RELEASE\n"); off(17);
  prepare(); dxl.devices[16].values[PRESENT_POSITION] = 4745 + 2049;
  command("ALIGNBL\n"); assert(!A::active && dxl.writes.empty());
  prepare(); dxl.devices[17].values[PRESENT_POSITION] = 3627 - 2049;
  command("ALIGNBR\n"); assert(!A::active && dxl.writes.empty());
  prepare(); A::begin(11);
  assert(!A::ramWrite(A::motors[0], HOMING_OFFSET, 0));
  assert(!A::ramWrite(A::motors[0], GOAL_POSITION, 0));
  assert(!A::ramWrite(A::motors[0], GOAL_PWM, 885));
  A::release(); off(11);
  prepare(); dxl.devices[19].values[TORQUE_ENABLE] = 1; A::startupRelease();
  for (auto w : dxl.writes) assert(w.item == TORQUE_ENABLE && w.value == 0 && w.id >= 11 && w.id <= 19);
  assert(dxl.devices[19].values[TORQUE_ENABLE] == 0);
  puts("SQuRo assembly checks passed: scoped RAM moves, goal-before-torque, tracking, aborts, timeouts, release failures and command parser.");
}
