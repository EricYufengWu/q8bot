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
    // q8Dynamixel(uint8_t firstID, int baud, HardwareSerial ser);
    q8Dynamixel(HardwareSerial ser);
    // void begin();
    // void enableTorque();
    // void disableTorque();
    // void setOperatingMode();
    // void moveAll(int32_t val);
    // void bulkRead();
    // void bulkWrite();
    
    Dynamixel2Arduino _dxl;

  private:
    // static const uint8_t _DXL_ID_CNT = 8;
    // uint8_t _DXL[_DXL_ID_CNT];
    // const uint8_t _DXL_DIR_PIN = 8;
    // const float _DXL_PROTOCOL_VERSION = 2.0;
    // static const uint16_t _user_pkt_buf_cap = 128;
    // uint8_t _user_pkt_buf[_user_pkt_buf_cap];
    // int _baudrate;

    // // Struct definitions for br (bulk read) and bw (bulk write)
    // struct br_data_xel{
    //   int32_t present_position;
    // } __attribute__((packed));
    // struct bw_data_xel{
    //   int32_t goal_position;
    // } __attribute__((packed));

    // struct br_data_xel _br_data_xel[_DXL_ID_CNT];
    // DYNAMIXEL::InfoBulkReadInst_t _br_infos;
    // DYNAMIXEL::XELInfoBulkRead_t _info_xels_br[_DXL_ID_CNT];
    // struct bw_data_xel _bw_data_xel[_DXL_ID_CNT]; 
    // DYNAMIXEL::InfoBulkWriteInst_t _bw_infos;
    // DYNAMIXEL::XELInfoBulkWrite_t _info_xels_bw[_DXL_ID_CNT];

    // uint16_t dxl2Deg();
    // int32_t deg2Dxl();
    // void setProfile();
};

#endif