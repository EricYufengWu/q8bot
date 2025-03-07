/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <q8Dynamixel.h>

q8Dynamixel::q8Dynamixel(HardwareSerial ser){
  // for (int i = 0; i < _DXL_ID_CNT; i++){
  //   _DXL[i] = 11 + i;
  // }
  // _baudrate = 1000000;

  Dynamixel2Arduino _dxl(ser, 8); 
}

// void q8Dynamixel::begin(){
//   _dxl.begin(_baudrate);

//   // fill the members of structure for bulkRead using external user packet buffer
//   _br_infos.packet.p_buf = _user_pkt_buf;
//   _br_infos.packet.buf_capacity = _user_pkt_buf_cap;
//   _br_infos.packet.is_completed = false;
//   _br_infos.p_xels = _info_xels_br;
//   _br_infos.xel_count = 0;

//   for (int i = 0; i < _DXL_ID_CNT; i++){
//     _info_xels_br[i].id = _DXL[i];
//     _info_xels_br[i].addr = 132; // Present Position of X serise.
//     _info_xels_br[i].addr_length = 4; // Present Current + Position + Velocity
//     _info_xels_br[i].p_recv_buf = reinterpret_cast<uint8_t*>(&_br_data_xel[i]);
//     _br_infos.xel_count++;
//   }
//   _br_infos.is_info_changed = true;

//   // Fill the members of structure for bulkWrite using internal packet buffer
//   _bw_infos.packet.p_buf = nullptr;
//   _bw_infos.packet.is_completed = false;
//   _bw_infos.p_xels = _info_xels_bw;
//   _bw_infos.xel_count = 0;

//   for (int i = 0; i < _DXL_ID_CNT; i++){
//     _bw_data_xel[i].goal_position = 0;
//     _info_xels_bw[i].id = _DXL[i];
//     _info_xels_bw[i].addr = 116; // Goal Position of X serise.
//     _info_xels_bw[i].addr_length = 4; // Goal Position
//     _info_xels_bw[i].p_data = reinterpret_cast<uint8_t*>(&_bw_data_xel[i]);
//     _bw_infos.xel_count++;
//   }
//   _bw_infos.is_info_changed = true;

//   // for Time-based Extended Pos, Profile velocity is the move duration (ms).
//   for (int i = 0; i < _DXL_ID_CNT; i++){
//     _dxl.writeControlTableItem(PROFILE_VELOCITY, _DXL[i], 1000);
//     _dxl.writeControlTableItem(PROFILE_ACCELERATION, _DXL[i], 300);
//   }
// }

// void q8Dynamixel::enableTorque(){
//   for (int i = 0; i < _DXL_ID_CNT; i++){
//     _dxl.torqueOn(_DXL[i]);
//   }
// }

// void q8Dynamixel::disableTorque(){
//   for (int i = 0; i < _DXL_ID_CNT; i++){
//     _dxl.torqueOff(_DXL[i]);
//   }
// }

// void q8Dynamixel::setOperatingMode(){
//   // Set the correct operating mode for all Dynamixels
//   enableTorque();
//   for (int i = 0; i < _DXL_ID_CNT; i++){
//     _dxl.setOperatingMode(_DXL[i], OP_EXTENDED_POSITION);
//   }
//   disableTorque();
// }

// void q8Dynamixel::moveAll(int32_t _val){
//   for (int i = 0; i < _DXL_ID_CNT; i++){
//     _bw_data_xel[i].goal_position = _val;
//   }
//   _bw_infos.is_info_changed = true;

//   _dxl.bulkWrite(&_bw_infos);
// }

// void q8Dynamixel::bulkRead(){

// }

// void q8Dynamixel::bulkWrite(){

// }

// uint16_t q8Dynamixel::dxl2Deg(){
//   return 0;
// }

// int32_t q8Dynamixel::deg2Dxl(){
//   return 0;
// }

// void q8Dynamixel::setProfile(){

// }