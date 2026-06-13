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

//Please see eManual Control Table section of your DYNAMIXEL.
//This example is written for DYNAMIXEL AX & MX series with Protocol 1.0.
//For MX 2.0 with Protocol 2.0, refer to write_x.ino example.
#define CW_ANGLE_LIMIT_ADDR         6
#define CCW_ANGLE_LIMIT_ADDR        8
#define ANGLE_LIMIT_ADDR_LEN        2
#define OPERATING_MODE_ADDR_LEN     2
#define TORQUE_ENABLE_ADDR          24
#define TORQUE_ENABLE_ADDR_LEN      1
#define LED_ADDR                    25
#define LED_ADDR_LEN                1
#define GOAL_POSITION_ADDR          30
#define GOAL_POSITION_ADDR_LEN      2
#define PRESENT_POSITION_ADDR       36
#define PRESENT_POSITION_ADDR_LEN   2
#define TIMEOUT 10    //default communication timeout 10ms

uint8_t turn_on = 1;
uint8_t turn_off = 0;

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 1.0;

uint16_t goalPosition1 = 0;
uint16_t goalPosition2 = 1023;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);   //Set debugging port baudrate to 115200bps
  while(!DEBUG_SERIAL);         //Wait until the serial port for terminal is opened
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Turn off torque when configuring items in EEPROM area
  if(dxl.write(DXL_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_off , TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("DYNAMIXEL Torque off");
  else
    DEBUG_SERIAL.println("Error: Torque off failed");

  // Set to Joint Mode
  if(dxl.write(DXL_ID, CW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition1, ANGLE_LIMIT_ADDR_LEN, TIMEOUT)
        && dxl.write(DXL_ID, CCW_ANGLE_LIMIT_ADDR, (uint8_t*)&goalPosition2, ANGLE_LIMIT_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Set operating mode");
  else
    DEBUG_SERIAL.println("Error: Set operating mode failed");

  // Turn on torque
  if(dxl.write(DXL_ID, TORQUE_ENABLE_ADDR, (uint8_t*)&turn_on, TORQUE_ENABLE_ADDR_LEN, TIMEOUT))
    DEBUG_SERIAL.println("Torque on");
  else
    DEBUG_SERIAL.println("Error: Torque on failed");
}

void loop() {
  // put your main code here, to run repeatedly:

  // LED On
  DEBUG_SERIAL.println("LED ON");
  dxl.write(DXL_ID, LED_ADDR, (uint8_t*)&turn_on, LED_ADDR_LEN, TIMEOUT);
  delay(500);
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position
  DEBUG_SERIAL.print("Goal Position : ");
  DEBUG_SERIAL.println(goalPosition1);
  dxl.write(DXL_ID, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition1, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  delay(1000);
  
  // LED Off
  DEBUG_SERIAL.println("LED OFF");
  dxl.write(DXL_ID, LED_ADDR, (uint8_t*)&turn_off, LED_ADDR_LEN, TIMEOUT);
  delay(500);

  // Set Goal Position
  DEBUG_SERIAL.print("Goal Position : ");
  DEBUG_SERIAL.println(goalPosition2);
  dxl.write(DXL_ID, GOAL_POSITION_ADDR, (uint8_t*)&goalPosition2, GOAL_POSITION_ADDR_LEN, TIMEOUT);
  delay(1000);
}
