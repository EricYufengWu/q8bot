// Host-side checks of the real configuration sequence, with a simulated bus.
// c++ -std=c++17 tests/test_replacement.cpp -o /tmp/q8-replacement-test && /tmp/q8-replacement-test
#include <cassert>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <string>
#include <vector>
enum {
  ID, BAUD_RATE, DRIVE_MODE, OPERATING_MODE, HOMING_OFFSET, TORQUE_ENABLE,
  MODEL_NUMBER, HARDWARE_ERROR_STATUS, PRESENT_INPUT_VOLTAGE, PRESENT_TEMPERATURE,
  PRESENT_POSITION, GOAL_POSITION, GOAL_PWM, PWM_LIMIT, PROFILE_VELOCITY,
  PROFILE_ACCELERATION, BUS_WATCHDOG, PRESENT_CURRENT, PRESENT_PWM
};
uint32_t fakeClock = 0;
uint32_t millis() { return fakeClock; }
void delay(unsigned long ms) { fakeClock += ms; }
struct FakeSerial {
  std::string output;
  void println(const char* s) { output += std::string(s) + '\n'; }
  void printf(const char* format, ...) {
    char b[512]; va_list args; va_start(args, format);
    vsnprintf(b, sizeof(b), format, args); va_end(args); output += b;
  }
} Serial;
struct Device {
  uint32_t baud = 57600;
  std::map<int,int32_t> values = {{ID,1}, {BAUD_RATE,1}, {DRIVE_MODE,0},
    {OPERATING_MODE,3}, {HOMING_OFFSET,0}, {TORQUE_ENABLE,0}, {MODEL_NUMBER,1190},
    {HARDWARE_ERROR_STATUS,0}, {PRESENT_INPUT_VOLTAGE,49}, {PRESENT_TEMPERATURE,25},
    {PRESENT_POSITION,2048}, {GOAL_POSITION,2048}, {GOAL_PWM,885}, {PWM_LIMIT,885},
    {PROFILE_VELOCITY,0}, {PROFILE_ACCELERATION,0}, {BUS_WATCHDOG,0},
    {PRESENT_CURRENT,0}, {PRESENT_PWM,0}};
};
struct Write { uint8_t item, id; int32_t value; };
struct FakeDxl {
  uint32_t baud = 1000000;
  std::map<uint8_t,Device> devices;
  std::vector<Write> writes;
  int lastError = 0, failedItem = -1;
  bool lostChangeAck = false;
  FakeDxl() {
    for (uint8_t id = 11; id <= 17; ++id) {
      Device d; d.baud = 1000000; d.values[ID] = id; d.values[BAUD_RATE] = 3;
      d.values[OPERATING_MODE] = 4; d.values[DRIVE_MODE] = 4;
      d.values[HOMING_OFFSET] = 2048; devices[id] = d;
    }
    devices[1] = Device{};
  }
  void begin(uint32_t value) { baud = value; }
  bool ping(uint8_t id) { return devices.count(id) && devices.at(id).baud == baud; }
  int32_t readControlTableItem(uint8_t item, uint8_t id, uint32_t) {
    lastError = ping(id) ? 0 : 3;
    return lastError ? 0 : devices.at(id).values[item];
  }
  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t) {
    writes.push_back({item,id,value});
    if (!ping(id) || item == failedItem) { lastError = 3; return false; }
    Device d = devices.at(id);
    if ((item == ID || item == BAUD_RATE || item == OPERATING_MODE ||
         item == DRIVE_MODE || item == HOMING_OFFSET) && d.values[TORQUE_ENABLE] != 0) {
      lastError = 4; return false;
    }
    d.values[item] = value;
    if (item == ID) { devices.erase(id); devices[static_cast<uint8_t>(value)] = d; }
    else {
      if (item == BAUD_RATE) d.baud = value == 3 ? 1000000 : 57600;
      devices[id] = d;
    }
    lastError = lostChangeAck && (item == ID || item == BAUD_RATE) ? 3 : 0;
    return lastError == 0;
  }
  int getLastLibErrCode() { return lastError; }
  int getLastStatusPacketError() { return 0; }
} dxl;
#include "../src/replacement_config.h"
void reset() { dxl = FakeDxl{}; Serial = FakeSerial{}; fakeClock = 0; }
void scoped() {
  for (auto w : dxl.writes) {
    assert(w.id == 1 || w.id == 18);
    assert((w.item == TORQUE_ENABLE && w.value == 0) ||
           (w.item == OPERATING_MODE && w.value == 4) ||
           (w.item == DRIVE_MODE && w.value == 5) ||
           (w.item == HOMING_OFFSET && w.value == -4096) ||
           (w.item == ID && w.id == 1 && w.value == 18) ||
           (w.item == BAUD_RATE && w.id == 18 && w.value == 3));
  }
}
#ifndef Q8_ALIGNMENT_TEST
int main() {
  reset(); Replacement18::status(); assert(dxl.writes.empty());
  for (int targetId : {1,18}) for (int speed : {57600,1000000}) {
    reset(); auto d = dxl.devices.at(1); dxl.devices.erase(1);
    d.baud = speed; d.values[ID] = targetId; d.values[BAUD_RATE] = speed == 57600 ? 1 : 3;
    dxl.devices[static_cast<uint8_t>(targetId)] = d;
    assert(Replacement18::configure()); assert(Replacement18::verifyTarget()); scoped();
    size_t count = dxl.writes.size();
    assert(Replacement18::configure()); assert(dxl.writes.size() == count);
  }
  reset(); dxl.lostChangeAck = true; assert(Replacement18::configure()); scoped();
  reset(); dxl.devices[1].values[TORQUE_ENABLE] = 1;
  assert(Replacement18::configure()); scoped();
  assert(dxl.writes.front().item == TORQUE_ENABLE && dxl.writes.front().value == 0);
  for (int missing : {1,11,12,13,14,15,16,17}) {
    reset(); dxl.devices.erase(static_cast<uint8_t>(missing));
    assert(!Replacement18::configure()); assert(dxl.writes.empty());
  }
  reset(); dxl.devices[18] = Device{};
  assert(!Replacement18::configure()); assert(dxl.writes.empty());
  for (auto invalid : std::vector<std::pair<int,int>>{{MODEL_NUMBER,1200},
      {HARDWARE_ERROR_STATUS,32}, {PRESENT_INPUT_VOLTAGE,34},
      {PRESENT_INPUT_VOLTAGE,56}, {PRESENT_TEMPERATURE,45}}) {
    reset(); dxl.devices[1].values[invalid.first] = invalid.second;
    assert(!Replacement18::configure()); assert(dxl.writes.empty());
  }
  reset(); dxl.devices[17].values[TORQUE_ENABLE] = 1;
  assert(!Replacement18::configure()); assert(dxl.writes.empty());
  for (int item : {TORQUE_ENABLE,OPERATING_MODE,DRIVE_MODE,HOMING_OFFSET,ID,BAUD_RATE}) {
    reset(); dxl.failedItem = item;
    if (item == TORQUE_ENABLE) dxl.devices[1].values[TORQUE_ENABLE] = 1;
    assert(!Replacement18::configure()); scoped();
  }
  reset(); assert(!Replacement18::put(TORQUE_ENABLE,1,1));
  assert(!Replacement18::put(DRIVE_MODE,11,5)); assert(dxl.writes.empty());
  puts("Replacement checks passed: four ID/baud states, readback, lost acknowledgments, repeat, failure guards, no movement or other-servo writes.");
}
#endif
