/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/

#include <cassert>

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <q8Dynamixel.h>
#include <esp_now.h>

#include "gait_lut.h"
#include "helpers.h"
#include "communications.h"
#include "systemParams.h"

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

void q8Dynamixel::resetTorqueState(){
  // Reset the internal torque flag to match disabled state
  // Used after connection loss when torque was already disabled
  _torqueFlag = false;
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
  if (recv_cnt == _idCount) {
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

void q8Dynamixel::handleDataMessage(ESPNowMessage msg){
  lastHeartbeatReceived = millis();  // Any DATA also counts as "alive"
  memcpy(&theirMsg, msg.data, sizeof(theirMsg));
  Message parsed = Message(theirMsg.data);

  // myMsg params
  myMsg.id = 0;  // Server ID

  switch (parsed.GetCommand()) {
    case MOVE: {
      assert(parsed.GetPayloadLen() == 8u);

      int32_t dxl[8];
      for (uint8_t i = 0; i < 8; ++i){
        dxl[i] = _deg2Dxl(parsed.GetPayloadFloats()[i]);
      }

      bulkWrite(dxl);
      break;
    }

    case BATTERY: {
      // Send battery level
      myMsg.data[0] = (uint16_t)FuelGauge.percent();
      esp_now_send(msg.mac, (uint8_t*)&myMsg, sizeof(myMsg));
      break;
    }

    case RECORD: {
      // Sync read position data
      uint16_t* posArray = syncRead();
      if (rData != nullptr) {
        // If rData is not nullptr, resize posArray to append new data
        uint16_t* newData = new uint16_t[masterSize + smallerSize];
        if (newData == nullptr) {
          queuePrint(MSG_DEBUG, "[ERROR] Memory allocation failed, dropping data\n");
          delete[] posArray;
          break;
        }
        memcpy(newData, rData, masterSize * sizeof(uint16_t));
        for (size_t j = 0; j < smallerSize; ++j) {
          newData[masterSize + j] = posArray[j];
        }
        delete[] rData;
        rData = newData;
        masterSize += smallerSize;
      } else {
        // If rData is nullptr, initialize it for the first time
        rData = new uint16_t[smallerSize];
        if (rData == nullptr) {
          queuePrint(MSG_DEBUG, "[ERROR] Memory allocation failed\n");
          delete[] posArray;
          break;
        }
        for (size_t j = 0; j < smallerSize; ++j) {
          rData[j] = posArray[j];
        }
        masterSize = smallerSize;
      }
      delete[] posArray;
      break;
    }

    case SEND_RECORDED: {
      // Send all recorded data in chunks
      if (rData != nullptr) {
        queuePrint(MSG_DEBUG, "[DATA] Sending %d recorded data points\n", masterSize);

        size_t totalSize = masterSize;  // total data in rData
        size_t chunkSize = 100;         // Chunk size for each ESPNow send
        size_t offset = 0;              // To track the position in rData

        // Loop to send the data in chunks
        while (offset < totalSize) {
          // If rData is less than chunk size then use its size instead.
          size_t currentChunkSize = (totalSize - offset < chunkSize) ? (totalSize - offset) : chunkSize;
          memcpy(myMsg.data, &rData[offset], currentChunkSize * sizeof(uint16_t));
          // If the chunk is smaller than 100, fill the remaining spaces with zeros
          if (currentChunkSize < chunkSize) {
            memset(&myMsg.data[currentChunkSize], 0, (chunkSize - currentChunkSize));
          }
          // Send the chunk via ESP-NOW
          esp_now_send(clientMac, (uint8_t*)&myMsg, sizeof(myMsg));
          // Update offset and Reset myData
          offset += currentChunkSize;
          memset(myMsg.data, 0, sizeof(myMsg.data));
        }
        delete[] rData;
        rData = nullptr;
        masterSize = 0;
      }

      break;
    }

    case JUMP: {
      jump();

      break;
    }

    case SET_TORQUE: {
      assert(parsed.GetPayloadLen() == 1u);
      _torqueFlag = bool(parsed.GetPayloadInts()[0]);

      if (_torqueFlag != _prevTorqueFlag){
        Serial.println(_torqueFlag ? "[ROBOT] Torque on" : "[ROBOT] Torque off");
        toggleTorque(_torqueFlag);
        _prevTorqueFlag = _torqueFlag;
      }

      break;
    }

    case SET_PROFILE: {
      assert(parsed.GetPayloadLen() == 1u);
      _profile = parsed.GetPayloadInts()[0];

      if (_profile != _prevProfile){
        Serial.print("[ROBOT] Profile changed: "); Serial.println(_profile);
        setProfile(_profile);
        _prevProfile = _profile;
      }

      break;
    }

    case SET_GAIT: {
      assert(parsed.GetPayloadLen() <= 1u);

      _gaitStepIdx = 0;
      if (parsed.GetPayloadLen() == 0){
        _using_gait = false;

        // Return to idle state
        queuePrint(MSG_DEBUG, "[GAIT] Returning to idle!\n");
        
        int32_t idle[8];
        for (uint8_t i = 0; i < 8; i++){
          idle[i] = _deg2Dxl(GaitIdleLUT[_gait][i]);
        }

        bulkWrite(idle);
      } else {
        _using_gait = true;
        _gait = static_cast<gait_t>(parsed.GetPayloadInts()[0]);
        queuePrint(MSG_DEBUG, "[GAIT] Gait set to %d\n", _gait);
      }
      
      break;
    }
  }
}

bool q8Dynamixel::getGaitActive() {
  return _using_gait;
}

void q8Dynamixel::performNextGaitMove() {
  if (!_using_gait) return;

  uint32_t offset = GaitLUTOffsets[_gait];
  
  uint32_t len = GaitLUTOffsets[_gait + 1] - offset;

  int32_t dxl[8];

  for (uint8_t i = 0; i < 8; i++){
    dxl[i] = _deg2Dxl(GaitLUT[offset + _gaitStepIdx][i]);
  }

  bulkWrite(dxl);

  _gaitStepIdx++;
  if (_gaitStepIdx >= len) {
    _gaitStepIdx = 0;
  }
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