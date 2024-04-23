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

int32_t q8Dynamixel::_deg2Dxl(float deg){
  return 0;
}

float q8Dynamixel::_dxl2Deg(int32_t dxlRaw){
  return 0.0;
}