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

  // Fill the members of structure to fastSyncRead using external user packet buffer
  _sr_infos.packet.buf_capacity = _user_pkt_buf_cap;
  _sr_infos.packet.p_buf = _user_pkt_buf;
  _sr_infos.packet.is_completed = false;
  _sr_infos.addr = SR_START_ADDR;
  _sr_infos.addr_length = SR_ADDR_LEN;
  _sr_infos.p_xels = _info_xels_sr;
  _sr_infos.xel_count = 0;  

  for(int i=0; i < _idCount; i++){
    _info_xels_sr[i].id = _DXL[i];;
    _info_xels_sr[i].p_recv_buf = (uint8_t*)&_sr_data[i];
    _sr_infos.xel_count++;
  }
  _sr_infos.is_info_changed = true;

  setProfile(1000);
  expandArrays();
}

bool q8Dynamixel::checkComms(uint8_t ID){
  return _dxl.ping(ID);
}

bool q8Dynamixel::commStart(){
  // Replace this with an actual check of ESPNow comms later
  return _torqueFlag;
}

uint16_t q8Dynamixel::checkBattery(){
  return 1;
}

void q8Dynamixel::enableTorque(){
  _dxl.torqueOn(BROADCAST_ID);
}

void q8Dynamixel::disableTorque(){
  _dxl.torqueOff(BROADCAST_ID);
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
  setGain(400);
  for (int i = 0; i < _idCount; i++){
    _dxl.writeControlTableItem(PROFILE_VELOCITY, _DXL[i], dur);
    _dxl.writeControlTableItem(PROFILE_ACCELERATION, _DXL[i], dur / 3);
  }
}

void q8Dynamixel::setGain(uint16_t p_gain){
  // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  for (int i = 0; i < _idCount; i++){
    _dxl.writeControlTableItem(POSITION_P_GAIN, _DXL[i], p_gain);
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

uint16_t* q8Dynamixel::syncRead(){
  // Read relevant registers from all joints into a single array
  int recv_cnt;
  size_t offset = 0;
  uint16_t* byteArray = new uint16_t[_idCount * 2];

  recv_cnt = _dxl.fastSyncRead(&_sr_infos);
  if(recv_cnt = _idCount){
    for (size_t i = 0; i < _idCount; i++){
      // cast to uint16_t since values never exceed 65535 in robot configuration
      byteArray[i*2] = static_cast<uint16_t>(_sr_data[i].present_current + 10000);
      byteArray[i*2+1] = static_cast<uint16_t>(_sr_data[i].present_position);
    }
  } else {
    // Fill ByteArray with zeros
    for (size_t i = 0; i < _idCount*2; ++i){
      byteArray[i] = 0;
    }
  }

  // for (size_t i = 0; i < 4; ++i){
  //   Serial.print(byteArray[i]); Serial.print(" ");
  // } Serial.println();
  return byteArray;
}

void q8Dynamixel::jump(){
  // Crouching Position
  setProfile(500);
  delay(100);
  bulkWrite(_lowArray);
  delay(1000);

  // Jump
  setProfile(0);
  setGain(800);
  delay(100);
  bulkWrite(_highArray);
  delay(100);
  bulkWrite(_restArray);
  delay(5000);

  // Back to idle
  setProfile(500);
  delay(100);
  bulkWrite(_idleArray);
  delay(1000);
  _prevProfile = 500;
}

uint8_t q8Dynamixel::parseData(const char* myData) {
  char* token = strtok(const_cast<char*>(myData), ",");
  int index = 0;
  int check = 0;
  
  while (token != nullptr && index < 8) {  // First 8 contain joint positions
    _posArray[index++] = _deg2Dxl(std::atof(token));
    token = strtok(nullptr, ",");
  }
  if (token != nullptr) {                  // 9th value is for special token
    _specialCmd = std::atoi(token);
    token = strtok(nullptr, ",");
    if (_specialCmd == 1){           // Battery
      return 1;
    } else if (_specialCmd == 2){    // Record
      check = 2;
    } else if (_specialCmd == 3){    // Send recorded
      return 3;
    } else if (_specialCmd == 4){    // Jump
      jump();
      return 0;
    }
  }
  if (token != nullptr) {                   // 10th value is vel/acc profiles
    _profile = std::atoi(token);
    token = strtok(nullptr, ",");
    if (_profile != _prevProfile){
      Serial.print("Profile changed: "); Serial.println(_profile);
      setProfile(_profile);
      _prevProfile = _profile;
    }
  }
  if (token != nullptr) {                    // 1th value is torque enable/disable
    _torqueFlag = (std::atoi(token) == 1);
    if (_torqueFlag != _prevTorqueFlag){
      Serial.println(_torqueFlag ? "Torque on" : "Torque off");
      toggleTorque(_torqueFlag);
      _prevTorqueFlag = _torqueFlag;
      return 0;
    }
  }
  bulkWrite(_posArray);
  return check - 0;
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

void q8Dynamixel::expandArrays(){
  for (int i = 0; i < 4; i++){
    _idleArray[i*2] = _deg2Dxl(_idlePos[0]);
    _idleArray[i*2+1] = _deg2Dxl(_idlePos[1]);
  }
  for (int i = 0; i < 4; i++){
    _lowArray[i*2] = _deg2Dxl(_jumpLow[0]);
    _lowArray[i*2+1] = _deg2Dxl(_jumpLow[1]);
  }
  for (int i = 0; i < 4; i++){
    _highArray[i*2] = _deg2Dxl(_jumpHigh[0]);
    _highArray[i*2+1] = _deg2Dxl(_jumpHigh[1]);
  }
  for (int i = 0; i < 4; i++){
    _restArray[i*2] = _deg2Dxl(_jumpRest[0]);
    _restArray[i*2+1] = _deg2Dxl(_jumpRest[1]);
  }
}