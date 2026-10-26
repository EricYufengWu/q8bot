/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/
#ifndef q8Dynamixel_h
#define q8Dynamixel_h

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

class q8Dynamixel
{
  public:
    q8Dynamixel(Dynamixel2Arduino& dxl);
    void begin();
    bool checkComms(uint8_t ID);
    bool commStart();
    uint16_t checkBattery();
    void enableTorque();
    void disableTorque();
    void toggleTorque(bool flag);
    void resetTorqueState();  // Reset internal torque flag without changing hardware state
    void setOpMode();
    void setProfile(uint16_t dur);
    void setGain(uint16_t p_gain);
    void moveSingle(int32_t val);
    void bulkWrite(int32_t values[8]);
    uint16_t* syncRead();
    void jump();
    uint8_t reportFaults();
    void recover();
    uint8_t parseData(const char* myData);
    // While a leg probe owns the servo bus, parseData serves only battery and
    // abort requests; every other packet is dropped without touching the bus.
    void lockMotion(bool locked) { _motionLocked = locked; }
    bool motionLocked() const { return _motionLocked; }
    const float* probeExcursion() const { return _probeExcursion; }
    int32_t probePwm() const { return _probePwm; }
    int32_t capPwm() const { return _capPwm; }
    float headDeg() const { return _headDeg; }
    uint32_t headMs() const { return _headMs; }

  private:
    Dynamixel2Arduino& _dxl; // Member variable to store the object of Dynamixel2Arduino

    const uint8_t BROADCAST_ID = 254;
    const uint16_t SR_START_ADDR = 126; //Present current, then velocity and position
    const uint16_t SR_ADDR_LEN = 10;
    const uint16_t SW_START_ADDR = 116; //Goal position
    const uint16_t SW_ADDR_LEN = 4;

    uint32_t _baudrate = 1000000;
    float _protocolVersion = 2.0;
    static const uint8_t _idCount = 8;
    const uint8_t _DXL[_idCount] = {11, 12, 13, 14, 15, 16, 17, 18};
    const uint8_t _directionPin = 8;
    static const uint16_t _user_pkt_buf_cap = 128;
    uint8_t _user_pkt_buf[_user_pkt_buf_cap];
    const int16_t _zeroOffset = 4096;
    const uint8_t _gearRatio = 1;
    int32_t _posArray[8];
    uint16_t _profile = 0;
    uint16_t _prevProfile = 0;
    bool _profileValid = false;
    uint32_t _lastProfileAttempt = 0;
    // 5 ms left no margin for bus contention; the library default is 100 ms.
    static const uint32_t _writeTimeout = 20;
    bool writeVerified(uint8_t item, uint8_t id, int32_t value);
    bool _torqueFlag = false;
    bool _prevTorqueFlag = false;
    uint8_t _specialCmd = 0;
    volatile bool _motionLocked = false;
    float _probeExcursion[8] = {0};
    int32_t _probePwm = 0;
    int32_t _capPwm = 0;
    float _headDeg = 0;
    uint32_t _headMs = 0;
    int32_t _deg2Dxl(float deg);
    float _dxl2Deg(int32_t dxlRaw);
    void expandArrays();
    const float _idlePos[2]   = {30, 150};
    int32_t _idleArray[8];
    const float _jumpLow[2]   = {-40, 220};
    int32_t _lowArray[8];
    const float _jumpHigh[2] = {90, 90};
    int32_t _highArray[8];
    const float _jumpRest[2]  = {30, 150};
    int32_t _restArray[8];

    // Struct definitions for br (bulk read) and bw (bulk write)
    struct br_data_xel{
      int32_t present_position;
    } __attribute__((packed));
    struct bw_data_xel{
      int32_t goal_position;
    } __attribute__((packed));

    // Struct definitions for sr (sync read) and sw (sync write)
    typedef struct sr_data{
      int16_t present_current;
      int32_t present_velocity;
      int32_t present_position;
    } __attribute__((packed)) sr_data_t;
    typedef struct sw_data{
      int32_t goal_position;
    } __attribute__((packed)) sw_data_t;

    struct br_data_xel _br_data_xel[_idCount];
    DYNAMIXEL::InfoBulkReadInst_t _br_infos;
    DYNAMIXEL::XELInfoBulkRead_t _info_xels_br[_idCount];
    struct bw_data_xel _bw_data_xel[_idCount]; 
    DYNAMIXEL::InfoBulkWriteInst_t _bw_infos;
    DYNAMIXEL::XELInfoBulkWrite_t _info_xels_bw[_idCount];

    sr_data_t _sr_data[_idCount];
    DYNAMIXEL::InfoSyncReadInst_t _sr_infos;
    DYNAMIXEL::XELInfoSyncRead_t _info_xels_sr[_idCount];
    sw_data_t _sw_data[_idCount];
    DYNAMIXEL::InfoSyncWriteInst_t _sw_infos;
    DYNAMIXEL::XELInfoSyncWrite_t _info_xels_sw[_idCount];
};

#endif
