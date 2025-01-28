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
    uint16_t checkVoltage();
    void enableTorque();
    void disableTorque();
    void toggleTorque(bool flag);
    void setOpMode();
    void setProfile(uint16_t dur);
    void moveSingle(int32_t val);
    void bulkWrite(int32_t values[8]);
    uint16_t* syncRead();
    void jump();
    void parseData(const char* myData);

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
    uint16_t _prevProfile;
    bool _torqueFlag = false;
    bool _prevTorqueFlag = false;
    uint8_t _specialCmd = 0;
    int32_t _deg2Dxl(float deg);
    float _dxl2Deg(int32_t dxlRaw);
    int32_t _idlePos[8]   = {4618, 5622, 4618, 5622, 4618, 5622, 4618, 5622};
    int32_t _jumpLow[8]   = {4221, 6019, 4221, 6019, 4221, 6019, 4221, 6019};
    int32_t _jumpHigh1[8] = {5177, 5063, 5177, 5063, 5177, 5063, 5177, 5063};
    int32_t _jumpHigh2[8] = {5120, 5120, 5120, 5120, 5120, 5120, 5120, 5120};
    int32_t _jumpHigh3[8] = {5063, 5177, 5063, 5177, 5063, 5177, 5063, 5177};
    int32_t _jumpHigh4[8] = {5006, 5234, 5006, 5234, 5006, 5234, 5006, 5234};
    int32_t _jumpRest[8]  = {4324, 5916, 4324, 5916, 4324, 5916, 4324, 5916};

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