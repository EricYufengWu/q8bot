/*
  legProbe.h - Gentle joint-range probing and telemetry for leg detection.

  A probe sweeps the requested joints a small step at a time with Goal PWM
  capped low, holds each moving joint's partner (the other servo of the same
  leg) where it already is, and stops a joint at the first of:
    - contact: the joint stops following its goal (stall), or draws excess
      current, which is the soft physical limit the host is looking for;
    - its excursion bound, which the host computes so that no sweep can reach
      a linkage change point of any candidate leg design. A parallelogram or
      five-bar passed through such a point can fold into its crossed form with
      almost no force, so a low PWM cap alone does not protect against it.

  Afterwards the swept joints return to where they started, torque goes off,
  and every RAM value that was changed is restored. Only RAM items (torque,
  Goal PWM, profiles, goal position) of IDs 11-18 are ever written.

  The class is templated on the bus so the host tests can drive it against a
  simulated Dynamixel bus; on the robot it wraps Dynamixel2Arduino.
*/
#ifndef legProbe_h
#define legProbe_h

#include <stdint.h>
#include <string.h>

namespace LegProbe {

constexpr uint8_t FIRST_ID = 11;
constexpr uint8_t JOINTS = 8;          // IDs 11-18
constexpr uint8_t SERVOS = 9;          // telemetry also reports the head, ID 19
constexpr uint8_t SLOTS = 9;           // probe slots: legs 0-7, head 8
constexpr uint8_t HEAD_SLOT = 8;       // ID 19; it has no partner joint
constexpr int32_t HEAD_PWM = 350;      // ~40 % output while the head is driven
constexpr float HEAD_LIMIT_DEG = 90.0f;   // hard clamp; the host keeps inside contact
constexpr uint16_t TAG_TELEMETRY = 61001;
constexpr uint16_t TAG_PROBE = 61002;
constexpr uint16_t TAG_CAP = 61003;
constexpr float TICKS_PER_DEG = 4096.0f / 360.0f;

constexpr uint32_t CYCLE_MS = 20;
constexpr int32_t STEP_TICKS = 9;          // ~0.8 deg per cycle, ~40 deg/s
constexpr int32_t STALL_LAG = 57;          // 5 deg behind the goal...
constexpr uint8_t STALL_CYCLES = 3;        // ...for three cycles in a row
constexpr int32_t CURRENT_LIMIT = 300;     // mA; a backup to the PWM cap
constexpr int32_t BACKOFF = 23;            // 2 deg of relief after contact
constexpr int32_t PARTNER_LIMIT = 57;      // 5 deg; beyond it the bounds are void
constexpr float MAX_EXCURSION_DEG = 150.0f;
constexpr int32_t PWM_MIN = 30, PWM_MAX = 250;   // 3.4% .. 28% duty
constexpr int32_t CAP_MIN = 100, CAP_MAX = 885;  // output cap for normal motion
constexpr uint8_t SETTLE_CYCLES = 10;      // at the bound, time to stop sagging
constexpr uint16_t EXTRA_CYCLES = 60;      // slack on top of the steps a sweep needs
constexpr uint32_t RETURN_MS = 1500;
constexpr int32_t RETURN_TOLERANCE = 23;   // 2 deg
constexpr uint32_t IO_TIMEOUT = 20;

// Per-joint outcome of a probe.
enum Reason : uint16_t { NONE = 0, CONTACT = 1, OVERCURRENT = 2, BOUND = 3, HELD = 4 };

// Overall outcome. Anything from ABORT_FAULT on releases torque at once,
// without returning the joints to their start.
enum Status : uint16_t {
  OK = 0, REFUSED_BUSY = 1, REFUSED_REQUEST = 2, REFUSED_TORQUE = 3,
  REFUSED_FAULT = 4, SETUP_FAILED = 5, ABORT_FAULT = 6, ABORT_READ = 7,
  ABORT_DEFLECTION = 8, ABORT_REQUEST = 9, ABORT_TIMEOUT = 10, RESTORE_FAILED = 11
};

inline uint16_t encode(int32_t value) {
  int32_t shifted = value + 32768;
  if (shifted < 0) shifted = 0;
  if (shifted > 65535) shifted = 65535;
  return static_cast<uint16_t>(shifted);
}

// The partner of a joint is the other servo of the same leg: 0<->1, 2<->3, ...
inline uint8_t partnerOf(uint8_t joint) { return joint ^ 1; }

template <typename Bus>
class Prober {
 public:
  explicit Prober(Bus& bus) : dxl(bus) {}

  // Set by the command task before a probe is queued, cleared when it ends.
  volatile bool busy = false;

  void requestAbort() { abortRequested = true; }

  // Reply layout: tag, servo count, then per servo ID 11..19
  // {flags (1 responding, 2 torque on, 4 hardware error), position+32768,
  //  current+32768, hardware error status}. Read-only.
  void telemetry(uint16_t out[100]) {
    memset(out, 0, 100 * sizeof(uint16_t));
    out[0] = TAG_TELEMETRY;
    out[1] = SERVOS;
    for (uint8_t i = 0; i < SERVOS; ++i) {
      uint8_t id = FIRST_ID + i;
      int32_t position = 0, current = 0, error = 0, torque = 0;
      bool ok = read(PRESENT_POSITION, id, position) && read(TORQUE_ENABLE, id, torque) &&
                read(PRESENT_CURRENT, id, current) && read(HARDWARE_ERROR_STATUS, id, error);
      uint16_t* slot = out + 2 + i * 4;
      slot[0] = (ok ? 1 : 0) | (ok && torque ? 2 : 0) | (ok && error ? 4 : 0);
      slot[1] = ok ? encode(position) : 0;
      slot[2] = ok ? encode(current) : 0;
      slot[3] = ok ? static_cast<uint16_t>(error) : 0;
    }
  }

  // Goal PWM cap on IDs 11-18 for normal motion, clamped to [CAP_MIN, PWM Limit].
  // Reply: tag, applied value, bit mask of servos that confirmed it.
  void applyCap(int32_t pwm, uint16_t out[100]) {
    memset(out, 0, 100 * sizeof(uint16_t));
    out[0] = TAG_CAP;
    if (pwm < CAP_MIN) pwm = CAP_MIN;
    if (pwm > CAP_MAX) pwm = CAP_MAX;
    uint16_t confirmed = 0;
    for (uint8_t i = 0; i < JOINTS; ++i) {
      uint8_t id = FIRST_ID + i;
      int32_t limit = CAP_MAX, value = pwm;
      if (read(PWM_LIMIT, id, limit) && limit < value) value = limit;
      if (write(GOAL_PWM, id, value)) confirmed |= (1u << i);
    }
    out[1] = static_cast<uint16_t>(pwm);
    out[2] = confirmed;
  }

  // excursion[i]: signed sweep bound in degrees for joint i (ID 11+i); 0 means
  // the joint does not sweep. Blocking; returns the status also placed in out.
  // Reply: tag, status, then per joint {flags (1 involved, 2 swept), reason,
  // start+32768, stop+32768, peak |current| mA, peak partner deflection ticks}.
  uint16_t run(const float excursion[JOINTS], int32_t pwm, uint16_t out[100]) {
    float request[SLOTS] = {0};
    memcpy(request, excursion, JOINTS * sizeof(float));
    return execute(request, pwm, false, out);
  }

  // The head on its own: sweep ID 19 by a signed bound to its contact and
  // back. The legs are not touched and may hold torque meanwhile.
  uint16_t runHead(float bound, int32_t pwm, uint16_t out[100]) {
    float request[SLOTS] = {0};
    request[HEAD_SLOT] = bound;
    return execute(request, pwm, true, out);
  }

  // Head yaw for normal use. Clamped to +-HEAD_LIMIT_DEG and driven at no
  // more than HEAD_PWM, so a bump is soft. If the head is released, its goal
  // is parked on the present position before torque comes on, so it cannot
  // jump; only ID 19 is torqued, the legs stay as they are.
  bool moveHead(float deg, uint32_t ms) {
    if (busy) return false;
    if (!(deg >= -HEAD_LIMIT_DEG)) deg = -HEAD_LIMIT_DEG;
    if (!(deg <= HEAD_LIMIT_DEG)) deg = HEAD_LIMIT_DEG;
    uint8_t id = FIRST_ID + HEAD_SLOT;
    int32_t torque = 0, error = 1;
    if (!read(TORQUE_ENABLE, id, torque) || !read(HARDWARE_ERROR_STATUS, id, error) || error) {
      return false;
    }
    if (ms > 5000) ms = 5000;
    if (!write(GOAL_PWM, id, HEAD_PWM)) return false;
    if (!torque) {
      int32_t present;
      if (!read(PRESENT_POSITION, id, present) || !write(GOAL_POSITION, id, present) ||
          !write(TORQUE_ENABLE, id, 1)) return false;
    }
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, static_cast<int32_t>(ms), IO_TIMEOUT);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, id, static_cast<int32_t>(ms / 4), IO_TIMEOUT);
    int32_t target = 4096 + static_cast<int32_t>(deg * TICKS_PER_DEG + (deg >= 0 ? 0.5f : -0.5f));
    return dxl.writeControlTableItem(GOAL_POSITION, id, target, IO_TIMEOUT);
  }

 private:
  uint16_t execute(const float request[SLOTS], int32_t pwm, bool head, uint16_t out[100]) {
    abortRequested = false;
    memset(joints, 0, sizeof(joints));
    uint16_t status = prepare(request, pwm, head);
    if (status == OK) status = sweep();
    uint16_t finish = settle(status == OK);
    if (status == OK && finish != OK) status = finish;
    if (finish == RESTORE_FAILED) status = RESTORE_FAILED;
    report(out, status);
    return status;
  }

 private:
  struct Joint {
    bool involved, swept, done, saved, torqued;
    int8_t dir;
    uint8_t lagCycles, limitCycles;
    uint16_t reason;
    int32_t start, goal, bound, stop;
    int32_t oldPwm, oldVelocity, oldAcceleration;
    int32_t peakCurrent, deflection;
  };

  Bus& dxl;
  volatile bool abortRequested = false;
  Joint joints[SLOTS];

  bool read(uint8_t item, uint8_t id, int32_t& value) {
    value = dxl.readControlTableItem(item, id, IO_TIMEOUT);
    // Bit 7 of the status error is the Alert flag for a latched hardware
    // error; the hardware error register itself is read separately.
    return dxl.getLastLibErrCode() == 0 && (dxl.getLastStatusPacketError() & 0x7f) == 0;
  }

  bool write(uint8_t item, uint8_t id, int32_t value) {
    if (id < FIRST_ID || id >= FIRST_ID + SLOTS) return false;
    if (item != TORQUE_ENABLE && item != GOAL_PWM && item != PROFILE_VELOCITY &&
        item != PROFILE_ACCELERATION && item != GOAL_POSITION) return false;
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
      if (!dxl.writeControlTableItem(item, id, value, IO_TIMEOUT)) continue;
      int32_t actual;
      if (read(item, id, actual) && actual == value) return true;
    }
    return false;
  }

  // Unverified goal update inside the sweep loop; positions are read back anyway.
  void setGoal(uint8_t joint, int32_t goal) {
    joints[joint].goal = goal;
    dxl.writeControlTableItem(GOAL_POSITION, FIRST_ID + joint, goal, IO_TIMEOUT);
  }

  uint16_t prepare(const float excursion[SLOTS], int32_t pwm, bool head) {
    bool any = false;
    for (uint8_t i = 0; i < SLOTS; ++i) {
      float e = excursion[i];
      if (!(e >= -MAX_EXCURSION_DEG && e <= MAX_EXCURSION_DEG)) return REFUSED_REQUEST;
      if (e != 0.0f) any = true;
      // A leg probe never moves the head, and a head probe never a leg.
      if (e != 0.0f && (i == HEAD_SLOT) != head) return REFUSED_REQUEST;
    }
    if (!any || pwm < PWM_MIN || pwm > PWM_MAX) return REFUSED_REQUEST;

    // Probing starts from released servos: no torque, no latched fault. A leg
    // probe needs all eight legs released; a head probe only the head.
    uint8_t first = head ? HEAD_SLOT : 0, last = head ? SLOTS : JOINTS;
    for (uint8_t i = first; i < last; ++i) {
      int32_t torque = 1, error = 1;
      if (!read(TORQUE_ENABLE, FIRST_ID + i, torque) ||
          !read(HARDWARE_ERROR_STATUS, FIRST_ID + i, error)) return REFUSED_FAULT;
      if (error != 0) return REFUSED_FAULT;
      if (torque != 0) return REFUSED_TORQUE;
    }

    // The host bounds each sweep for a partner held still. Sweeping both
    // joints of one leg at once would void that, so it is refused.
    for (uint8_t i = 0; i < JOINTS; i += 2) {
      if (excursion[i] != 0.0f && excursion[i + 1] != 0.0f) return REFUSED_REQUEST;
    }
    for (uint8_t i = 0; i < SLOTS; ++i) {
      if (excursion[i] == 0.0f) continue;
      joints[i].involved = joints[i].swept = true;
      if (i != HEAD_SLOT) joints[partnerOf(i)].involved = true;
      joints[i].dir = excursion[i] > 0 ? 1 : -1;
      float magnitude = excursion[i] > 0 ? excursion[i] : -excursion[i];
      joints[i].bound = static_cast<int32_t>(magnitude * TICKS_PER_DEG + 0.5f);
    }

    // Capture every value that will change before changing any of them.
    for (uint8_t i = 0; i < SLOTS; ++i) {
      Joint& j = joints[i];
      if (!j.involved) continue;
      uint8_t id = FIRST_ID + i;
      int32_t limit = PWM_MAX;
      if (!read(PRESENT_POSITION, id, j.start) || !read(GOAL_PWM, id, j.oldPwm) ||
          !read(PWM_LIMIT, id, limit) || !read(PROFILE_VELOCITY, id, j.oldVelocity) ||
          !read(PROFILE_ACCELERATION, id, j.oldAcceleration)) return SETUP_FAILED;
      j.saved = true;
      j.goal = j.stop = j.start;
      j.reason = j.swept ? NONE : HELD;
      int32_t cap = pwm < limit ? pwm : limit;
      // Goal is parked on the present position before torque, so arming
      // moves nothing.
      if (!write(GOAL_PWM, id, cap) || !write(PROFILE_VELOCITY, id, 0) ||
          !write(PROFILE_ACCELERATION, id, 0) || !write(GOAL_POSITION, id, j.start)) {
        return SETUP_FAILED;
      }
    }
    for (uint8_t i = 0; i < SLOTS; ++i) {
      if (!joints[i].involved) continue;
      joints[i].torqued = true;
      if (!write(TORQUE_ENABLE, FIRST_ID + i, 1)) return SETUP_FAILED;
    }
    return OK;
  }

  uint16_t sweep() {
    int32_t longest = 0;
    for (uint8_t i = 0; i < SLOTS; ++i) {
      if (joints[i].swept && joints[i].bound > longest) longest = joints[i].bound;
    }
    // Counted in cycles, not milliseconds: bus reads stretch every cycle
    // beyond CYCLE_MS, so a time budget would cut long sweeps short.
    uint32_t budget = static_cast<uint32_t>(longest / STEP_TICKS + 1) + EXTRA_CYCLES;
    for (uint32_t cycle = 0;; ++cycle) {
      if (abortRequested) return ABORT_REQUEST;
      if (cycle > budget) return ABORT_TIMEOUT;

      for (uint8_t i = 0; i < SLOTS; ++i) {
        Joint& j = joints[i];
        if (!j.swept || j.done) continue;
        int32_t limit = j.start + j.dir * j.bound;
        int32_t next = j.goal + j.dir * STEP_TICKS;
        if (j.dir * (next - limit) > 0) next = limit;
        if (next != j.goal) setGoal(i, next);
      }
      delay(CYCLE_MS);

      bool pending = false;
      for (uint8_t i = 0; i < SLOTS; ++i) {
        Joint& j = joints[i];
        if (!j.involved) continue;
        uint8_t id = FIRST_ID + i;
        int32_t position, current, error;
        if (!read(HARDWARE_ERROR_STATUS, id, error)) return ABORT_READ;
        if (error != 0) return ABORT_FAULT;
        if (!read(PRESENT_POSITION, id, position) || !read(PRESENT_CURRENT, id, current)) {
          return ABORT_READ;
        }
        int32_t magnitude = current < 0 ? -current : current;
        if (magnitude > j.peakCurrent) j.peakCurrent = magnitude;

        if (!j.swept) {
          // A partner pushed this far means the leg is no longer in the
          // configuration the host bounded the sweep for.
          int32_t moved = position - j.start;
          if (moved < 0) moved = -moved;
          if (moved > j.deflection) j.deflection = moved;
          if (moved > PARTNER_LIMIT) return ABORT_DEFLECTION;
          continue;
        }
        if (j.done) continue;
        int32_t lag = j.dir * (j.goal - position);
        bool atLimit = j.goal == j.start + j.dir * j.bound;
        if (magnitude > CURRENT_LIMIT) {
          stopAt(i, position, OVERCURRENT);
        } else if (lag > STALL_LAG) {
          if (++j.lagCycles >= STALL_CYCLES) stopAt(i, position, CONTACT);
        } else {
          j.lagCycles = 0;
          // At the bound a joint may sit a few degrees short under gravity;
          // give it a moment, then call the bound reached without contact.
          if (atLimit && (lag <= STALL_LAG / 2 || ++j.limitCycles >= SETTLE_CYCLES)) {
            stopAt(i, position, BOUND);
          }
        }
        if (!j.done) pending = true;
      }
      if (!pending) return OK;
    }
  }

  void stopAt(uint8_t joint, int32_t position, uint16_t reason) {
    Joint& j = joints[joint];
    j.done = true;
    j.stop = position;
    j.reason = reason;
    // Relieve a contact so the joint is not left pressing on the stop.
    if (reason == CONTACT || reason == OVERCURRENT) setGoal(joint, position - j.dir * BACKOFF);
  }

  // Normal end: return to the start, then torque off. Abort: torque off now.
  uint16_t settle(bool returnToStart) {
    bool anyTorqued = false;
    for (uint8_t i = 0; i < SLOTS; ++i) anyTorqued = anyTorqued || joints[i].torqued;
    if (anyTorqued && returnToStart) {
      for (uint8_t i = 0; i < SLOTS; ++i) {
        Joint& j = joints[i];
        if (!j.torqued) continue;
        uint8_t id = FIRST_ID + i;
        dxl.writeControlTableItem(PROFILE_VELOCITY, id, static_cast<int32_t>(RETURN_MS), IO_TIMEOUT);
        dxl.writeControlTableItem(PROFILE_ACCELERATION, id, static_cast<int32_t>(RETURN_MS / 4), IO_TIMEOUT);
        setGoal(i, j.start);
      }
      uint32_t began = millis();
      while (millis() - began < RETURN_MS + 1000 && !abortRequested) {
        delay(50);
        bool home = true;
        for (uint8_t i = 0; i < SLOTS; ++i) {
          if (!joints[i].torqued) continue;
          int32_t position;
          if (!read(PRESENT_POSITION, FIRST_ID + i, position)) { home = false; break; }
          int32_t error = position - joints[i].start;
          if (error < -RETURN_TOLERANCE || error > RETURN_TOLERANCE) home = false;
        }
        if (home) break;
      }
    }
    bool off = true;
    bool released[SLOTS];
    for (uint8_t i = 0; i < SLOTS; ++i) {
      released[i] = !joints[i].torqued || torqueOff(FIRST_ID + i);
      off = off && released[i];
    }
    // Restoring output and profiles on a servo that might still hold torque
    // could let it move at full output, so such a servo keeps the low probe
    // cap. Every confirmed-off servo is restored, so the robot is not left
    // weak on healthy joints; the failure is still reported.
    bool restored = off;
    for (uint8_t i = 0; i < SLOTS; ++i) {
      Joint& j = joints[i];
      if (!j.saved || !released[i]) continue;
      uint8_t id = FIRST_ID + i;
      restored = write(PROFILE_VELOCITY, id, j.oldVelocity) && restored;
      restored = write(PROFILE_ACCELERATION, id, j.oldAcceleration) && restored;
      restored = write(GOAL_PWM, id, j.oldPwm) && restored;
    }
    return restored ? OK : RESTORE_FAILED;
  }

  bool torqueOff(uint8_t id) {
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
      dxl.writeControlTableItem(TORQUE_ENABLE, id, 0, IO_TIMEOUT);
      int32_t value = 1;
      if (read(TORQUE_ENABLE, id, value) && value == 0) return true;
    }
    return false;
  }

  void report(uint16_t out[100], uint16_t status) {
    memset(out, 0, 100 * sizeof(uint16_t));
    out[0] = TAG_PROBE;
    out[1] = status;
    for (uint8_t i = 0; i < SLOTS; ++i) {
      const Joint& j = joints[i];
      uint16_t* slot = out + 2 + i * 6;
      slot[0] = (j.involved ? 1 : 0) | (j.swept ? 2 : 0);
      slot[1] = j.reason;
      slot[2] = j.involved ? encode(j.start) : 0;
      slot[3] = j.involved ? encode(j.stop) : 0;
      slot[4] = static_cast<uint16_t>(j.peakCurrent > 65535 ? 65535 : j.peakCurrent);
      slot[5] = static_cast<uint16_t>(j.deflection > 65535 ? 65535 : j.deflection);
    }
  }
};

}  // namespace LegProbe

#endif
