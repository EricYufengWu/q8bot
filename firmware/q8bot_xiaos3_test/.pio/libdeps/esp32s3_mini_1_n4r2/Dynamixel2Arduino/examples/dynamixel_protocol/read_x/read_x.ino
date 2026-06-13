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
//This example is written based on DYNAMIXEL X series(excluding XL-320)
#define ID_ADDR                 7
#define ID_ADDR_LEN             1
#define BAUDRATE_ADDR           8
#define BAUDRATE_ADDR_LEN       1
#define PROTOCOL_TYPE_ADDR      13
#define PROTOCOL_TYPE_ADDR_LEN  1
#define TIMEOUT 10    //default communication timeout 10ms

uint8_t returned_id = 0;
uint8_t returned_baudrate = 0;
uint8_t returned_protocol = 0;

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);   //Set debugging port baudrate to 115200bps
  while(!DEBUG_SERIAL);         //Wait until the serial port for terminal is opened
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.println("Refer to eManual for more details.");
  DEBUG_SERIAL.println("https://emanual.robotis.com/docs/en/dxl/");
  DEBUG_SERIAL.print("Read for PROTOCOL ");
  DEBUG_SERIAL.print(DXL_PROTOCOL_VERSION, 1);
  DEBUG_SERIAL.print(", ID ");
  DEBUG_SERIAL.println(DXL_ID);

  // Read DYNAMIXEL ID
  dxl.read(DXL_ID, ID_ADDR, ID_ADDR_LEN, (uint8_t*)&returned_id, sizeof(returned_id), TIMEOUT);
  DEBUG_SERIAL.print("ID : ");
  DEBUG_SERIAL.println(returned_id);
  delay(100);
  // Read DYNAMIXEL Baudrate
  dxl.read(DXL_ID, BAUDRATE_ADDR, BAUDRATE_ADDR_LEN, (uint8_t*)&returned_baudrate, sizeof(returned_baudrate), TIMEOUT);
  DEBUG_SERIAL.print("Baud Rate : ");
  DEBUG_SERIAL.println(returned_baudrate);
  delay(100);
  // Read DYNAMIXEL Protocol type
  dxl.read(DXL_ID, PROTOCOL_TYPE_ADDR, PROTOCOL_TYPE_ADDR_LEN, (uint8_t*)&returned_protocol, sizeof(returned_protocol), TIMEOUT);
  DEBUG_SERIAL.print("Protocol Type : ");
  DEBUG_SERIAL.println(returned_protocol);
}

void loop() {
  // put your main code here, to run repeatedly:
}
