/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <q8Dynamixel.h>

using namespace ControlTableItem;

// Constructor that takes an object of type Dynamixel2Arduino as an argument
q8Dynamixel::q8Dynamixel(Dynamixel2Arduino& dxl) : _dxl(dxl) {
  // Constructor implementation
  // Initialize any other members if needed
}

void q8Dynamixel::begin(){
  _dxl.begin(_baudrate);
  _dxl.setPortProtocolVersion(_protocolVersion);
  setOpMode();

  // fill the members of structure for bulkRead using external user packet buffer
  _br_infos.packet.p_buf = _user_pkt_buf;
  _br_infos.packet.buf_capacity = _user_pkt_buf_cap;
  _br_infos.packet.is_completed = false;
  _br_infos.p_xels = _info_xels_br;
  _br_infos.xel_count = 0;

  for (int i = 0; i < _idCount; i++){
    _info_xels_br[i].id = _DXL[i];
    _info_xels_br[i].addr = 132; // Present Position of X serise.
    _info_xels_br[i].addr_length = 4; // Present Current + Position + Velocity
    _info_xels_br[i].p_recv_buf = reinterpret_cast<uint8_t*>(&_br_data_xel[i]);
    _br_infos.xel_count++;
  }
  _br_infos.is_info_changed = true;

  // Fill the members of structure for bulkWrite using internal packet buffer
  _bw_infos.packet.p_buf = nullptr;
  _bw_infos.packet.is_completed = false;
  _bw_infos.p_xels = _info_xels_bw;
  _bw_infos.xel_count = 0;

  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = 0;
    _info_xels_bw[i].id = _DXL[i];
    _info_xels_bw[i].addr = 116; // Goal Position of X serise.
    _info_xels_bw[i].addr_length = 4; // Goal Position
    _info_xels_bw[i].p_data = reinterpret_cast<uint8_t*>(&_bw_data_xel[i]);
    _bw_infos.xel_count++;
  }
  _bw_infos.is_info_changed = true;

  // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  for (int i = 0; i < _idCount; i++){
    _dxl.writeControlTableItem(PROFILE_VELOCITY, _DXL[i], 1000);
    _dxl.writeControlTableItem(PROFILE_ACCELERATION, _DXL[i], 300);
  }
}

bool q8Dynamixel::checkComms(uint8_t ID){
  return _dxl.ping(ID);
}

void q8Dynamixel::enableTorque(){
  for (int i = 0; i < _idCount; i++){
    _dxl.torqueOn(_DXL[i]);
  }
}

void q8Dynamixel::disableTorque(){
  for (int i = 0; i < _idCount; i++){
    _dxl.torqueOff(_DXL[i]);
  }
}

void q8Dynamixel::setOpMode(){
  // Set the correct operating mode for all Dynamixels
  disableTorque();
  for (int i = 0; i < _idCount; i++){
    _dxl.setOperatingMode(_DXL[i], OP_EXTENDED_POSITION);
  }
  enableTorque();
}

void q8Dynamixel::moveAll(float deg){
  // Temporary function for testing only. Replace with bw in final code
  for (int i = 0; i < _idCount; i++){
    _dxl.setGoalPosition(_DXL[i], deg, UNIT_DEGREE);
  }
}

void q8Dynamixel::bulkWrite(int32_t val){
  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = val;
  }
  _bw_infos.is_info_changed = true;

  _dxl.bulkWrite(&_bw_infos);
}

int32_t q8Dynamixel::_deg2Dxl(float deg){
  return 0;
}

float q8Dynamixel::_dxl2Deg(int32_t dxlRaw){
  return 0.0;
}