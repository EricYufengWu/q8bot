// Host-side checks of the leg probe against a simulated Dynamixel bus.
// c++ -std=c++17 -Wall -Wextra -I include test_host/test_leg_probe.cpp -o /tmp/q8-probe && /tmp/q8-probe
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <set>
#include <vector>

enum : uint8_t {
  ID, BAUD_RATE, DRIVE_MODE, OPERATING_MODE, HOMING_OFFSET, PWM_LIMIT,
  TORQUE_ENABLE, HARDWARE_ERROR_STATUS, GOAL_PWM, PROFILE_ACCELERATION,
  PROFILE_VELOCITY, GOAL_POSITION, PRESENT_CURRENT, PRESENT_POSITION
};

uint32_t fakeClock = 0;
std::function<void()> onTick;
void physics(uint32_t ms);
uint32_t millis() { return fakeClock; }
void delay(uint32_t ms) {
  for (uint32_t t = 0; t < ms; ++t) { ++fakeClock; physics(1); }
  if (onTick) onTick();
}

struct Servo {
  std::map<uint8_t, int32_t> v = {
    {TORQUE_ENABLE, 0}, {HARDWARE_ERROR_STATUS, 0}, {GOAL_PWM, 885}, {PWM_LIMIT, 885},
    {PROFILE_ACCELERATION, 0}, {PROFILE_VELOCITY, 0}, {GOAL_POSITION, 4096},
    {PRESENT_CURRENT, 0}, {PRESENT_POSITION, 4096}};
  double position = 4096;
  int32_t wallLow = -100000, wallHigh = 100000;
  int dragPartner = 0;     // ticks of partner drag per tick moved past dragFrom
  int32_t dragFrom = 0;
  double sag = 0;          // ticks the joint falls short of its goal at rest
};

struct Write { uint8_t item, id; int32_t value; };

struct FakeBus {
  std::map<uint8_t, Servo> servos;
  std::vector<Write> writes;
  std::set<uint8_t> unreadable;
  int lastError = 0;
  FakeBus() { for (uint8_t id = 11; id <= 19; ++id) servos[id] = Servo{}; }
  int32_t readControlTableItem(uint8_t item, uint8_t id, uint32_t) {
    if (!servos.count(id) || unreadable.count(id)) { lastError = 3; return 0; }
    lastError = 0;
    Servo& s = servos.at(id);
    if (item == PRESENT_POSITION) return static_cast<int32_t>(std::lround(s.position));
    return s.v.at(item);
  }
  bool writeControlTableItem(uint8_t item, uint8_t id, int32_t value, uint32_t) {
    writes.push_back({item, id, value});
    if (!servos.count(id)) { lastError = 3; return false; }
    lastError = 0;
    if (item == GOAL_PWM && value > servos[id].v[PWM_LIMIT]) return false;
    servos[id].v[item] = value;
    return true;
  }
  int getLastLibErrCode() { return lastError; }
  int getLastStatusPacketError() { return 0; }
} dxl;

// Joints track their goal at a speed scaled by Goal PWM, stop at walls, and
// draw current in proportion to the output cap while pressing on a wall.
void physics(uint32_t ms) {
  for (auto& [id, s] : dxl.servos) {
    if (!s.v[TORQUE_ENABLE]) { s.v[PRESENT_CURRENT] = 0; continue; }
    double rate = 4.0 * s.v[GOAL_PWM] / 885.0;           // ticks per ms
    double target = s.v[GOAL_POSITION];
    double gap = target - s.position;
    double rest = gap > 0 ? std::max(0.0, gap - s.sag) : std::min(0.0, gap + s.sag);
    double step = std::max(-rate * ms, std::min(rate * ms, rest));
    double before = s.position;
    s.position = std::max<double>(s.wallLow, std::min<double>(s.wallHigh, s.position + step));
    bool pressing = (target > s.wallHigh && s.position >= s.wallHigh) ||
                    (target < s.wallLow && s.position <= s.wallLow);
    s.v[PRESENT_CURRENT] = pressing ? s.v[GOAL_PWM] / 2 : 4;
    if (s.dragPartner && s.position > s.dragFrom) {
      uint8_t partner = 11 + ((id - 11) ^ 1);
      dxl.servos[partner].position += s.dragPartner * (s.position - std::max<double>(before, s.dragFrom));
    }
  }
}

#include "legProbe.h"
using namespace LegProbe;
Prober<FakeBus> prober(dxl);

const float DEG = 4096.0f / 360.0f;
uint16_t out[100];

void reset(int32_t start = 4600) {
  dxl = FakeBus{};
  for (uint8_t id = 11; id <= 19; ++id) {
    dxl.servos[id].position = start + (id - 11) * 10;
    dxl.servos[id].v[GOAL_POSITION] = 1234;           // stale goal from an earlier run
    dxl.servos[id].v[PROFILE_VELOCITY] = 1000;
  }
  onTick = nullptr;
  fakeClock = 0;
}
int32_t decoded(uint16_t value) { return static_cast<int32_t>(value) - 32768; }
uint16_t* slot(uint8_t joint) { return out + 2 + joint * 6; }

// No write outside RAM motion items or outside IDs 11-18, ever.
void scopeOk() {
  for (auto& w : dxl.writes) {
    assert(w.id >= 11 && w.id <= 18);
    assert(w.item == TORQUE_ENABLE || w.item == GOAL_PWM || w.item == PROFILE_VELOCITY ||
           w.item == PROFILE_ACCELERATION || w.item == GOAL_POSITION);
  }
}
// After any probe: all torque off, output and profiles back as they were.
void releasedAndRestored() {
  for (uint8_t id = 11; id <= 18; ++id) {
    assert(dxl.servos[id].v[TORQUE_ENABLE] == 0);
    assert(dxl.servos[id].v[GOAL_PWM] == 885);
    assert(dxl.servos[id].v[PROFILE_VELOCITY] == 1000);
  }
  scopeOk();
}

int main() {
  // Telemetry reports every servo, including the head, read-only.
  reset();
  dxl.servos[13].v[TORQUE_ENABLE] = 1;
  dxl.servos[16].v[HARDWARE_ERROR_STATUS] = 32;
  dxl.servos.erase(18);
  prober.telemetry(out);
  assert(out[0] == TAG_TELEMETRY && out[1] == 9);
  assert(out[2 + 0 * 4] == 1 && decoded(out[2 + 0 * 4 + 1]) == 4600);
  assert(out[2 + 2 * 4] == (1 | 2));
  assert(out[2 + 5 * 4] == (1 | 4) && out[2 + 5 * 4 + 3] == 32);
  assert(out[2 + 7 * 4] == 0);                        // ID18 missing
  assert(decoded(out[2 + 8 * 4 + 1]) == 4680);        // head, ID19
  assert(dxl.writes.empty());

  // Contact: every hind and front elbow (q1, even joints) meets a wall 40 deg
  // along its sweep. Each stops at the wall, returns, and releases.
  reset();
  float sweep[8] = {100, 0, 100, 0, 100, 0, 100, 0};
  for (uint8_t i = 0; i < 8; i += 2) {
    Servo& s = dxl.servos[11 + i];
    s.wallHigh = static_cast<int32_t>(s.position + 40 * DEG);
  }
  assert(prober.run(sweep, 140, out) == OK);
  assert(out[0] == TAG_PROBE && out[1] == OK);
  for (uint8_t i = 0; i < 8; i += 2) {
    int32_t start = decoded(slot(i)[2]), stop = decoded(slot(i)[3]);
    assert(slot(i)[0] == 3 && slot(i)[1] == CONTACT);
    assert(std::abs(stop - (start + static_cast<int32_t>(40 * DEG))) <= 3);
    assert(slot(i)[4] > 0);
    // The partner was held, not swept, and barely moved.
    assert(slot(i + 1)[0] == 1 && slot(i + 1)[1] == HELD && slot(i + 1)[5] <= 2);
    assert(std::abs(dxl.servos[11 + i].position - start) <= RETURN_TOLERANCE);
  }
  releasedAndRestored();
  // Torque came on only after the stale goal was parked at the present position.
  for (uint8_t id = 11; id <= 18; ++id) {
    bool parked = false;
    for (auto& w : dxl.writes) {
      if (w.id != id) continue;
      if (w.item == GOAL_POSITION && !parked) parked = true;
      if (w.item == TORQUE_ENABLE && w.value == 1) { assert(parked); break; }
    }
  }

  // Bound: no wall, so each joint reaches its bound and is reported as such,
  // in the negative direction as well.
  reset();
  float back[8] = {0, -30, 0, -30, 0, -30, 0, -30};
  assert(prober.run(back, 140, out) == OK);
  for (uint8_t i = 1; i < 8; i += 2) {
    int32_t start = decoded(slot(i)[2]), stop = decoded(slot(i)[3]);
    assert(slot(i)[1] == BOUND);
    assert(std::abs(stop - (start - static_cast<int32_t>(30 * DEG))) <= 3);
  }
  releasedAndRestored();

  // Gravity sag at the bound still ends as BOUND, not as a timeout.
  reset();
  dxl.servos[11].sag = 40;                             // 3.5 deg short
  float one[8] = {20, 0, 0, 0, 0, 0, 0, 0};
  assert(prober.run(one, 140, out) == OK && slot(0)[1] == BOUND);
  releasedAndRestored();

  // A held partner pushed out of place voids the bounds: stop at once.
  reset();
  dxl.servos[11].dragPartner = 4;   // a linkage push stronger than the low-output hold
  dxl.servos[11].dragFrom = static_cast<int32_t>(dxl.servos[11].position + 10 * DEG);
  uint16_t status = prober.run(one, 140, out);
  assert(status == ABORT_DEFLECTION && out[1] == ABORT_DEFLECTION);
  assert(slot(1)[5] > PARTNER_LIMIT);
  releasedAndRestored();

  // An abort request mid-sweep releases without the return move.
  reset();
  onTick = [] { if (fakeClock > 400) prober.requestAbort(); };
  float far[8] = {120, 0, 0, 0, 0, 0, 0, 0};
  assert(prober.run(far, 140, out) == ABORT_REQUEST);
  releasedAndRestored();

  // A hardware error mid-sweep aborts as a fault.
  reset();
  onTick = [] { if (fakeClock > 300) dxl.servos[12].v[HARDWARE_ERROR_STATUS] = 4; };
  assert(prober.run(far, 140, out) == ABORT_FAULT);
  for (uint8_t id = 11; id <= 18; ++id) assert(dxl.servos[id].v[TORQUE_ENABLE] == 0);

  // A servo that stops answering mid-sweep cannot confirm torque-off. That
  // outranks the read failure: the host must tell the user to cut power.
  // Every servo that did confirm torque-off is still restored.
  reset();
  onTick = [] { if (fakeClock > 300) dxl.unreadable.insert(11); };
  assert(prober.run(far, 140, out) == RESTORE_FAILED && out[1] == RESTORE_FAILED);
  dxl.unreadable.clear();
  assert(dxl.servos[11].v[GOAL_PWM] == 140);           // unconfirmed: keeps the low cap
  assert(dxl.servos[12].v[TORQUE_ENABLE] == 0 && dxl.servos[12].v[GOAL_PWM] == 885);
  scopeOk();

  // Refusals happen before anything is written.
  float both[8] = {20, 20, 0, 0, 0, 0, 0, 0};
  float none[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  float wide[8] = {151, 0, 0, 0, 0, 0, 0, 0};
  for (auto* request : {both, none, wide}) {
    reset();
    assert(prober.run(request, 140, out) == REFUSED_REQUEST && dxl.writes.empty());
  }
  reset();
  assert(prober.run(one, 10, out) == REFUSED_REQUEST && dxl.writes.empty());
  reset();
  assert(prober.run(one, 900, out) == REFUSED_REQUEST && dxl.writes.empty());
  reset();
  dxl.servos[17].v[TORQUE_ENABLE] = 1;
  assert(prober.run(one, 140, out) == REFUSED_TORQUE && dxl.writes.empty());
  reset();
  dxl.servos[14].v[HARDWARE_ERROR_STATUS] = 1;
  assert(prober.run(one, 140, out) == REFUSED_FAULT && dxl.writes.empty());

  // The output cap is clamped and respects each servo's PWM Limit.
  reset();
  dxl.servos[15].v[PWM_LIMIT] = 500;
  prober.applyCap(600, out);
  assert(out[0] == TAG_CAP && out[1] == 600 && out[2] == 0xFF);
  assert(dxl.servos[11].v[GOAL_PWM] == 600 && dxl.servos[15].v[GOAL_PWM] == 500);
  assert(dxl.servos[19].v[GOAL_PWM] == 885);
  prober.applyCap(5, out);
  assert(out[1] == CAP_MIN);
  scopeOk();

  // Head probe: finds contact each way, writes nothing but ID 19, leaves the
  // legs alone even if they hold torque (a standing robot), and releases.
  for (int direction : {1, -1}) {
    reset();
    for (uint8_t id = 11; id <= 18; ++id) dxl.servos[id].v[TORQUE_ENABLE] = 1;
    Servo& head = dxl.servos[19];
    if (direction > 0) head.wallHigh = static_cast<int32_t>(head.position + 35 * DEG);
    else head.wallLow = static_cast<int32_t>(head.position - 28 * DEG);
    assert(prober.runHead(direction * 90.0f, 140, out) == OK);
    uint16_t* h = out + 2 + 8 * 6;
    assert(h[0] == 3 && h[1] == CONTACT);
    int32_t expected = static_cast<int32_t>((direction > 0 ? 35 : -28) * DEG);
    assert(std::abs(decoded(h[3]) - decoded(h[2]) - expected) <= 3);
    for (auto& w : dxl.writes) assert(w.id == 19);
    assert(head.v[TORQUE_ENABLE] == 0 && head.v[GOAL_PWM] == 885 && head.v[PROFILE_VELOCITY] == 1000);
    for (uint8_t id = 11; id <= 18; ++id) assert(dxl.servos[id].v[TORQUE_ENABLE] == 1);
  }
  // A torqued head is not probed; a head probe cannot carry leg sweeps.
  reset();
  dxl.servos[19].v[TORQUE_ENABLE] = 1;
  assert(prober.runHead(40.0f, 140, out) == REFUSED_TORQUE && dxl.writes.empty());
  // A leg probe never touches the head.
  reset();
  assert(prober.run(one, 140, out) == OK);
  for (auto& w : dxl.writes) assert(w.id != 19);

  // Head yaw: parks the goal before torque, caps output, clamps to the hard
  // limit, touches only ID 19, and stays off the bus while a probe runs.
  reset();
  dxl.servos[19].position = 4096 - 50;
  assert(prober.moveHead(20.0f, 80));
  bool parked = false;
  for (auto& w : dxl.writes) {
    assert(w.id == 19);
    if (w.item == GOAL_POSITION && w.value == 4096 - 50) parked = true;
    if (w.item == TORQUE_ENABLE && w.value == 1) assert(parked);
  }
  assert(dxl.servos[19].v[GOAL_PWM] == HEAD_PWM);
  assert(dxl.servos[19].v[GOAL_POSITION] == 4096 + static_cast<int32_t>(20 * DEG + 0.5f));
  assert(prober.moveHead(170.0f, 80));
  assert(dxl.servos[19].v[GOAL_POSITION] == 4096 + static_cast<int32_t>(HEAD_LIMIT_DEG * DEG + 0.5f));
  assert(prober.moveHead(-170.0f, 80));
  assert(dxl.servos[19].v[GOAL_POSITION] == 4096 - static_cast<int32_t>(HEAD_LIMIT_DEG * DEG + 0.5f));
  for (uint8_t id = 11; id <= 18; ++id) assert(dxl.servos[id].v[TORQUE_ENABLE] == 0);
  size_t before = dxl.writes.size();
  prober.busy = true;
  assert(!prober.moveHead(0.0f, 80) && dxl.writes.size() == before);
  prober.busy = false;
  dxl.servos[19].v[HARDWARE_ERROR_STATUS] = 4;
  assert(!prober.moveHead(0.0f, 80) && dxl.writes.size() == before);

  std::puts("Leg probe checks passed: telemetry, contact, bound, sag, partner hold and deflection, "
            "abort, fault, read loss, refusals, release/restore, write scope, output cap, "
            "head probe, head yaw.");
  return 0;
}
