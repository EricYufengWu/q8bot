#define Q8_ALIGNMENT_TEST
#include "test_replacement.cpp"
#include "../src/replacement_alignment.h"

void prepare() {
  reset(); Alignment18::active = false; Alignment18::holding = false;
  assert(Replacement18::configure());
  dxl.devices[18].values[PRESENT_POSITION] = 6002;
  dxl.devices[18].values[GOAL_POSITION] = 6002;
  dxl.writes.clear(); Serial.output.clear();
}
void checkOff() {
  assert(!Alignment18::active && dxl.devices[18].values[TORQUE_ENABLE] == 0);
  assert(dxl.devices[18].values[GOAL_PWM] == 885);
  for (auto w : dxl.writes) {
    assert(w.id == 18);
    assert(w.item == TORQUE_ENABLE || w.item == GOAL_PWM || w.item == PROFILE_VELOCITY ||
           w.item == PROFILE_ACCELERATION || w.item == GOAL_POSITION);
    if (w.item == GOAL_POSITION) assert(w.value == 6002 || w.value == 5622);
  }
}
int main() {
  prepare(); Alignment18::begin(); assert(Alignment18::active);
  assert(dxl.devices[18].values[TORQUE_ENABLE] == 1);
  assert(dxl.devices[18].values[GOAL_PWM] == 266);
  assert(dxl.devices[18].values[GOAL_POSITION] == 5622);
  uint32_t started = millis();
  for (int n = 0; n < 90; ++n) {
    delay(50); uint32_t elapsed = millis() - started;
    dxl.devices[18].values[PRESENT_POSITION] = elapsed >= 4000 ? 5622 : 6002 - 380 * elapsed / 4000;
    Alignment18::tick();
  }
  assert(Alignment18::active && Alignment18::holding);
  assert(Serial.output.find("ALIGN18 READY") != std::string::npos);
  size_t count = dxl.writes.size(); Alignment18::begin(); assert(dxl.writes.size() == count);
  delay(300000); Alignment18::tick(); checkOff();

  prepare(); Alignment18::begin(); Alignment18::release(); checkOff();
  prepare(); Alignment18::begin();
  for (int n = 0; n < 150; ++n) { delay(50); Alignment18::tick(); }
  checkOff(); assert(Serial.output.find("not reached in time") != std::string::npos);

  for (auto invalid : std::vector<std::pair<int,int>>{{HARDWARE_ERROR_STATUS,32},
      {PRESENT_CURRENT,251}, {PRESENT_CURRENT,-251}, {PRESENT_PWM,267},
      {PRESENT_INPUT_VOLTAGE,34}, {PRESENT_INPUT_VOLTAGE,56},
      {PRESENT_TEMPERATURE,45}, {PRESENT_POSITION,6200}, {TORQUE_ENABLE,0}}) {
    prepare(); Alignment18::begin(); dxl.devices[18].values[invalid.first] = invalid.second;
    delay(50); Alignment18::tick(); checkOff();
  }
  for (int item : {GOAL_PWM,PROFILE_VELOCITY,PROFILE_ACCELERATION,GOAL_POSITION}) {
    prepare(); dxl.failedItem = item; Alignment18::begin();
    assert(!Alignment18::active && dxl.devices[18].values[TORQUE_ENABLE] == 0);
    for (auto w : dxl.writes) assert(w.id == 18);
  }
  prepare(); dxl.devices[18].values[DRIVE_MODE] = 4;
  Alignment18::begin(); assert(!Alignment18::active && dxl.writes.empty());
  prepare(); dxl.devices[17].values[TORQUE_ENABLE] = 1;
  Alignment18::begin(); assert(!Alignment18::active && dxl.writes.empty());
  prepare(); Alignment18::begin();
  assert(!Alignment18::ramWrite(GOAL_PWM,885));
  assert(!Alignment18::ramWrite(HOMING_OFFSET,0));
  assert(!Alignment18::ramWrite(GOAL_POSITION,0));
  Alignment18::release(); checkOff();
  puts("Alignment checks passed: target/hold, timeout, faults, release, write failures, preflight and ID18-only RAM scope.");
}
