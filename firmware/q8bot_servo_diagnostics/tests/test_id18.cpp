// Host-side checks of the actual test sequence with a simulated servo bus.
// Run: c++ -std=c++17 tests/test_id18.cpp -o /tmp/q8-id18-test && /tmp/q8-id18-test
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#ifndef Q8_TEST_ID
#define Q8_TEST_ID 18
#endif

enum {
  TORQUE_ENABLE, GOAL_PWM, PROFILE_VELOCITY, PROFILE_ACCELERATION, GOAL_POSITION,
  HARDWARE_ERROR_STATUS, PRESENT_POSITION, PRESENT_CURRENT, PRESENT_PWM,
  PRESENT_INPUT_VOLTAGE, PRESENT_TEMPERATURE, OPERATING_MODE, DRIVE_MODE,
  HOMING_OFFSET, BUS_WATCHDOG, PWM_LIMIT
};
uint32_t fakeTime = 0;
uint32_t millis() { return fakeTime; }
void delay(uint32_t ms) { fakeTime += ms; }
struct FakeSerial {
  std::string input, output;
  int available() { return static_cast<int>(input.size()); }
  int read() { char c = input.front(); input.erase(0, 1); return c; }
  void println(const char* s) { output += std::string(s) + '\n'; }
  void printf(const char* format, ...) {
    char b[512]; va_list args; va_start(args, format);
    vsnprintf(b, sizeof(b), format, args); va_end(args); output += b;
  }
} Serial;
struct Write { uint8_t item, id; int32_t value; };
struct FakeDxl {
  std::map<int, int32_t> values = {
    {TORQUE_ENABLE, 0}, {GOAL_PWM, 885}, {PROFILE_VELOCITY, 0},
    {PROFILE_ACCELERATION, 0}, {GOAL_POSITION, 6073}, {PRESENT_POSITION, 6073},
    {HARDWARE_ERROR_STATUS, 0}, {PRESENT_CURRENT, 0}, {PRESENT_PWM, 0},
    {PRESENT_INPUT_VOLTAGE, 49}, {PRESENT_TEMPERATURE, 27},
    {OPERATING_MODE, 4}, {DRIVE_MODE, Q8_TEST_ID == 18 ? 5 : 4},
    {HOMING_OFFSET, Q8_TEST_ID == 18 ? -4096 : 4096},
    {BUS_WATCHDOG, 0}, {PWM_LIMIT, 885}
  };
  std::vector<Write> writes;
  int libError = 0, status = 0, absent = -1, failedWrite = -1;
  bool otherTorque = false, overcurrent = false, alert = false, activeReadFailure = false;
  bool failTorqueOff = false;
  int32_t readControlTableItem(uint8_t item, uint8_t id, uint32_t) {
    ++fakeTime;
    libError = id == absent ? 3 : 0;
    bool on = values[TORQUE_ENABLE] == 1;
    if (activeReadFailure && on && item == PRESENT_POSITION) libError = 3;
    status = alert && on ? 0x80 : 0;
    if (id != Q8_TEST_ID) return item == TORQUE_ENABLE && otherTorque ? 1 : 0;
    if (item == PRESENT_POSITION && on) values[item] = values[GOAL_POSITION];
    if (item == PRESENT_CURRENT && on && overcurrent) return 300;
    return values[item];
  }
  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t) {
    writes.push_back({item, id, value}); libError = 0; status = 0;
    if (item == failedWrite || (failTorqueOff && item == TORQUE_ENABLE && value == 0)) {
      libError = 3; return false;
    }
    values[item] = value;
    return true;
  }
  int getLastLibErrCode() { return libError; }
  int getLastStatusPacketError() { return status; }
} dxl;

#include "../src/id18_test.h"

void reset() { dxl = FakeDxl{}; Serial = FakeSerial{}; fakeTime = 0; }
bool enabled() {
  for (auto w : dxl.writes) if (w.item == TORQUE_ENABLE && w.value == 1) return true;
  return false;
}
void assertScopedAndOff() {
  assert(dxl.values[TORQUE_ENABLE] == 0);
  for (auto w : dxl.writes) {
    assert(w.id == Q8_TEST_ID);
    assert(w.item == TORQUE_ENABLE || w.item == GOAL_PWM ||
           w.item == PROFILE_VELOCITY || w.item == PROFILE_ACCELERATION || w.item == GOAL_POSITION);
    if (w.item == GOAL_POSITION) assert(w.value >= 6039 && w.value <= 6073);
  }
}
int main() {
  reset(); Id18Test::run(); assert(enabled()); assertScopedAndOff();
  assert(fakeTime < 3500 && fakeTime >= 2800);
  assert(dxl.values[GOAL_PWM] == 885 && dxl.values[PROFILE_VELOCITY] == 0);
  assert(dxl.values[PROFILE_ACCELERATION] == 0);
  assert(Serial.output.find(std::string(Id18Test::COMMAND) + " FINISHED") != std::string::npos);
  bool sawOut = false, sawReturn = false, limitedBeforeEnable = false;
  for (auto w : dxl.writes) {
    if (w.item == GOAL_PWM && w.value == 133) limitedBeforeEnable = true;
    if (w.item == TORQUE_ENABLE && w.value == 1) assert(limitedBeforeEnable);
    if (w.item == GOAL_POSITION && w.value == 6039) sawOut = true;
    if (sawOut && w.item == GOAL_POSITION && w.value == 6073) sawReturn = true;
  }
  assert(sawOut && sawReturn);
  for (int id = 11; id <= 18; ++id) {
    reset(); dxl.absent = id; Id18Test::run(); assert(dxl.writes.empty());
  }
  for (auto invalid : std::vector<std::pair<int,int>>{{OPERATING_MODE, 3}, {DRIVE_MODE, 13},
       {HOMING_OFFSET, 0}, {BUS_WATCHDOG, -1}, {HARDWARE_ERROR_STATUS, 32},
       {PRESENT_INPUT_VOLTAGE, 34}, {PRESENT_INPUT_VOLTAGE, 56}, {PRESENT_TEMPERATURE, 45},
       {PRESENT_POSITION, 6251}, {PRESENT_POSITION, 4399}, {GOAL_PWM, 0}}) {
    reset(); dxl.values[invalid.first] = invalid.second; Id18Test::run(); assert(dxl.writes.empty());
  }
  reset(); dxl.otherTorque = true; Id18Test::run(); assert(dxl.writes.empty());
  for (int item : {GOAL_PWM, PROFILE_VELOCITY, PROFILE_ACCELERATION, GOAL_POSITION}) {
    reset(); dxl.failedWrite = item; Id18Test::run(); assert(!enabled()); assertScopedAndOff();
  }
  for (int fault = 0; fault < 4; ++fault) {
    reset();
    dxl.overcurrent = fault == 0; dxl.alert = fault == 1; dxl.activeReadFailure = fault == 2;
    if (fault == 3) Serial.input = "!";
    Id18Test::run(); assert(enabled()); assertScopedAndOff();
    assert(Serial.output.find(std::string(Id18Test::COMMAND) + " ABORTED") != std::string::npos);
    assert(fakeTime < 500);
  }
  reset(); dxl.values[GOAL_PWM] = 80; Id18Test::run(); assertScopedAndOff();
  for (auto w : dxl.writes) if (w.item == GOAL_PWM) assert(w.value <= 80);
  reset(); dxl.failTorqueOff = true; Id18Test::run();
  assert(Serial.output.find("TORQUE-OFF NOT CONFIRMED") != std::string::npos);
  assert(dxl.values[GOAL_PWM] == 133);  // Never restore full output while still on.
  reset(); Serial.input = "garbage\nxxxxxxxxxxxxxxxxTEST18\n";
  pollId18TestCommand(); assert(dxl.writes.empty());
  reset(); Serial.input = std::string(Id18Test::COMMAND) + '\n' + Id18Test::COMMAND + '\n'; pollId18TestCommand();
  int enables = 0;
  for (auto w : dxl.writes) if (w.item == TORQUE_ENABLE && w.value == 1) ++enables;
  assert(enables == 1); assertScopedAndOff();
  printf("ID%d sequence checks passed: nominal, preflight, bounded output, failures, abort, cleanup, command parsing.\n", Q8_TEST_ID);
}
