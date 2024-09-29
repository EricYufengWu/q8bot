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

  setProfile(1000);
}

bool q8Dynamixel::checkComms(uint8_t ID){
  return _dxl.ping(ID);
}

bool q8Dynamixel::commStart(){
  // Replace this with an actual check of ESPNow comms later
  return _torqueFlag;
}

uint16_t q8Dynamixel::checkVoltage(){
  return _dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, _DXL[0]);
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

void q8Dynamixel::toggleTorque(bool flag){
  if(flag){
    enableTorque();
  } else{
    disableTorque();
  }
}

void q8Dynamixel::setOpMode(){
  // Set operating mode. Torque off first if needed.
  if (!_torqueFlag){
    for (int i = 0; i < _idCount; i++){
      _dxl.setOperatingMode(_DXL[i], OP_EXTENDED_POSITION);
    }
  } else{
    disableTorque();
    for (int i = 0; i < _idCount; i++){
      _dxl.setOperatingMode(_DXL[i], OP_EXTENDED_POSITION);
    }
    enableTorque();
  }
}

void q8Dynamixel::setProfile(uint16_t dur){
  // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  for (int i = 0; i < _idCount; i++){
    _dxl.writeControlTableItem(PROFILE_VELOCITY, _DXL[i], dur);
    _dxl.writeControlTableItem(PROFILE_ACCELERATION, _DXL[i], dur / 3);
  }
}

void q8Dynamixel::moveSingle(int32_t val){
  // 8 motors move to the same position
  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = val;
  }
  _bw_infos.is_info_changed = true;

  _dxl.bulkWrite(&_bw_infos);
}

void q8Dynamixel::bulkWrite(int32_t values[8]){
  // 8 motors move to their respective positions
  for (int i = 0; i < _idCount; i++){
    _bw_data_xel[i].goal_position = values[i];
  }
  _bw_infos.is_info_changed = true;

  _dxl.bulkWrite(&_bw_infos);
}

void q8Dynamixel::jump(){
  // float _degArray1[8] = {45.9,134.1,45.9,134.1,45.9,134.1,45.9,134.1};
  // for (int i = 0; i < 8; i++){
  //   Serial.print(_deg2Dxl(_degArray1[i])); Serial.print(" ");
  // } Serial.println();

  setProfile(500);
  delay(100);
  Serial.println("Jumping Low");
  bulkWrite(_jumpRest);
  setProfile(0);
  delay(1000);
  Serial.println("Jumping High");
  bulkWrite(_jumpHigh1);
  delay(100);
  Serial.println("Jumping Rest");
  bulkWrite(_jumpRest);
  delay(1000);
  setProfile(500);
  delay(1000);
  Serial.println("Idle");
  bulkWrite(_idlePos);
  _prevProfile = 500;
}

void q8Dynamixel::parseData(const char* myData) {
  char* token = strtok(const_cast<char*>(myData), ",");
  int index = 0;
  
  while (token != nullptr && index < 8) {
    _posArray[index++] = _deg2Dxl(std::atof(token));
    token = strtok(nullptr, ",");
  }
  if (token != nullptr) {     // special
    _specialCmd = std::atoi(token);
    token = strtok(nullptr, ",");
    if (_specialCmd == 1){
      Serial.println("Check battery");
      return;
    } else if (_specialCmd == 2){
      jump();
      // Serial.println("Jumping");
      return;
    }
  }
  if (token != nullptr) {     // move profile
    _profile = std::atoi(token);
    token = strtok(nullptr, ",");
    if (_profile != _prevProfile){
      Serial.print("Profile changed: "); Serial.println(_profile);
      setProfile(_profile);
      _prevProfile = _profile;
    }
  }
  if (token != nullptr) {     // torque
    _torqueFlag = (std::atoi(token) == 1);
    if (_torqueFlag != _prevTorqueFlag){
      Serial.println(_torqueFlag ? "Torque on" : "Torque off");
      toggleTorque(_torqueFlag);
      _prevTorqueFlag = _torqueFlag;
      return;
    }
  }
  bulkWrite(_posArray);
}

int32_t q8Dynamixel::_deg2Dxl(float deg){
  // Dynamixel joint 0 to 360 deg is 0 to 4096
  const float friendlyPerDxl = 360.0 / 4096.0 / _gearRatio;
  int angleDxl = static_cast<int>(deg / friendlyPerDxl + 0.5) + _zeroOffset;
  return angleDxl;
}

float q8Dynamixel::_dxl2Deg(int32_t dxlRaw){
  // Dynamixel joint 0 to 360 deg is 0 to 4096
  const float friendlyPerDxl = 360.0 / 4096.0 / _gearRatio;
  float angleFriendly = (dxlRaw - _zeroOffset) * friendlyPerDxl;
  return angleFriendly;
}