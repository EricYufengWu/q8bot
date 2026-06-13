/*******************************************************************************
* Copyright 2022 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

// Tutorial Video: https://youtu.be/msWlMyx8Nrw
// Example Environment
//
// - DYNAMIXEL: X series
//              ID = 1, Baudrate = 57600bps, DYNAMIXEL Protocol 2.0
// - Controller: Arduino MKR ZERO
//               DYNAMIXEL Shield for Arduino MKR
// - https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/
// - Adjust the position_p_gain, position_i_gain, position_d_gain values
// Author: David Park

#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

const uint8_t DXL_ID = 1;
const uint32_t DXL_BAUDRATE = 57600;
const float DXL_PROTOCOL_VERSION = 2.0;

int32_t goal_position[2] = {1200, 1600};
int8_t direction = 0;
unsigned long timer = 0;

// Position PID Gains
// Adjust these gains to tune the behavior of DYNAMIXEL
uint16_t position_p_gain = 0;
uint16_t position_i_gain = 0;
uint16_t position_d_gain = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  // For Uno, Nano, Mini, and Mega, use the UART port of the DYNAMIXEL Shield to read debugging messages.
  DEBUG_SERIAL.begin(57600);
  while(!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(DXL_BAUDRATE);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
  
  // Set Position PID Gains
  dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID, position_p_gain);
  dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID, position_i_gain);
  dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID, position_d_gain);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Read Present Position (Use the Serial Plotter)
  while(true) {
    DEBUG_SERIAL.print("Goal_Position:");
    DEBUG_SERIAL.print(dxl.readControlTableItem(GOAL_POSITION, DXL_ID));
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print("Present_Position:");
    DEBUG_SERIAL.print(dxl.getPresentPosition(DXL_ID));
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.println();
    delay(10);

    if (millis() - timer >= 2000) {
      dxl.setGoalPosition(DXL_ID, goal_position[direction]);
      timer = millis();
      break;
    }
  }

  if(direction >= 1) {
    direction = 0;
  } else {
    direction = 1;
  }
}
