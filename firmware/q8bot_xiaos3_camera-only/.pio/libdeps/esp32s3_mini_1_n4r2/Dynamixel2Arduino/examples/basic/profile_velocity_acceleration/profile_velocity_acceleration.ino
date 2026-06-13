/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

/* NOTE: Please check if your DYNAMIXEL supports Profile Control
* Supported Products : MX-28(2.0), MX-64(2.0), MX-106(2.0), All X series(except XL-320)
* PRO(A) series / PRO+ series
*/

#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif
 

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;   

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);
  
  // Turn on the LED on DYNAMIXEL
  dxl.ledOn(DXL_ID);
  delay(500);
  // Turn off the LED on DYNAMIXEL
  dxl.ledOff(DXL_ID);
  delay(500);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);
  dxl.torqueOn(DXL_ID);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t key_input;
  DEBUG_SERIAL.println("Select Profile Type :");
  DEBUG_SERIAL.println("[1] Low Accel / High Vel");
  DEBUG_SERIAL.println("[2] Max Accel / Low Vel");
  DEBUG_SERIAL.println("[3] Max Accel / Max Vel");
  
  while(DEBUG_SERIAL.available()==0);
  key_input = DEBUG_SERIAL.read();

  switch(key_input) {
    case '1':
      // Low Profile Acceleration, High Profile Velocity
      // Refer to API documentation for available parameters
      // http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/#dynamixelshieldv010-or-above
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 50);
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 300);
      break;
    case '2':
      // Max Profile Acceleration, Low Profile Velocity
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 0);
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 100);
      break;
    case '3':
      // Max Profile Acceleration, Max Profile Velocity
      // WARNING : Please BE AWARE that this option will make a vibrant motion for PRO(A) or PRO+ series that requires high current supply.
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 0);
      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 0);
      break;
    default:
      break;
  }

  // Set Goal Position in RAW value
  dxl.setGoalPosition(DXL_ID, 0);
  delay(500);
  // Check if DYNAMIXEL is in motion
  while(dxl.readControlTableItem(MOVING, DXL_ID));
  
  // Set Goal Position in angle(degree)
  dxl.setGoalPosition(DXL_ID, 179.0, UNIT_DEGREE);
  delay(500);
  // Check if DYNAMIXEL is in motion
  while(dxl.readControlTableItem(MOVING, DXL_ID));
}
