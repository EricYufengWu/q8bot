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

/*
  This is an example deprecated. (Not recommended, just an example for legacy)
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

/* ParamForSyncReadInst_t
  A structure that contains the information needed for the parameters of the 'syncRead packet'.

  typedef struct ParamForSyncReadInst{
    uint16_t addr;
    uint16_t length;
    uint8_t id_count;
    InfoForSyncReadParam_t xel[DXL_MAX_NODE]; //refer to below.
  } ParamForSyncReadInst_t;

  typedef struct InfoForSyncReadParam{
    uint8_t id;
  } InfoForSyncReadParam_t;
*/

/* ParamForSyncWriteInst_t
  A structure that contains the information needed for the parameters of the 'syncWrite packet'.

  typedef struct ParamForSyncWriteInst{
    uint16_t addr;
    uint16_t length;
    uint8_t id_count;
    XelInfoForSyncWriteParam_t xel[DXL_MAX_NODE]; //refer to below.
  } ParamForSyncWriteInst_t;

  typedef struct XelInfoForSyncWriteParam{
    uint8_t id;
    uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
  } XelInfoForSyncWriteParam_t;
*/

/* RecvInfoFromStatusInst_t
  A structure used to receive data from multiple XELs.

  typedef struct RecvInfoFromStatusInst{
    uint8_t id_count;
    XelInfoForStatusInst_t xel[DXL_MAX_NODE]; //refer to below.
  } RecvInfoFromStatusInst_t;

  typedef struct XelInfoForStatusInst{
    uint8_t id;
    uint16_t length;
    uint8_t error;
    uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
  } XelInfoForStatusInst_t;
*/

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.scan();

  // fill the members of structure for syncWrite
  sync_write_param.addr = 104; //Goal Velocity on X serise
  sync_write_param.length = 4;
  sync_write_param.xel[0].id = 1;
  sync_write_param.xel[1].id = 2;
  sync_write_param.id_count = 2;

  // fill the members of structure for syncRead
  sync_read_param.addr = 128; //Present Velocity on X serise
  sync_read_param.length = 4;
  sync_read_param.xel[0].id = 1;
  sync_read_param.xel[1].id = 2;
  sync_read_param.id_count = 2;
  
  dxl.torqueOff(1);
  dxl.setOperatingMode(1, OP_VELOCITY);
  dxl.torqueOn(1);

  dxl.torqueOff(3);
  dxl.setOperatingMode(3, OP_VELOCITY);
  dxl.torqueOn(3);
}

void loop() {
  // put your main code here, to run repeatedly:
  static int32_t velocity = 0;
  int32_t recv_velocity[2];

  // set value to data buffer for syncWrite
  velocity = velocity >= 200 ? -200 : velocity+10;
  memcpy(sync_write_param.xel[0].data, &velocity, sizeof(velocity));
  memcpy(sync_write_param.xel[1].data, &velocity, sizeof(velocity));

  // send command using syncWrite
  dxl.syncWrite(sync_write_param);
  delay(100);

  // Print the read data using SyncRead
  dxl.syncRead(sync_read_param, read_result);
  DEBUG_SERIAL.println(F("======= Sync Read ======="));
  memcpy(&recv_velocity[0], read_result.xel[0].data, read_result.xel[0].length);
  memcpy(&recv_velocity[1], read_result.xel[1].data, read_result.xel[1].length);
  DEBUG_SERIAL.print(F("ID: "));DEBUG_SERIAL.print(read_result.xel[0].id);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(F(", Present Velocity: "));DEBUG_SERIAL.print(recv_velocity[0]);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(F(", Packet Error: "));DEBUG_SERIAL.print(read_result.xel[0].error);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(F(", Param Length: "));DEBUG_SERIAL.print(read_result.xel[0].length);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print(F("ID: "));DEBUG_SERIAL.print(read_result.xel[1].id);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(F(", Present Velocity: "));DEBUG_SERIAL.print(recv_velocity[1]);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(F(", Packet Error: "));DEBUG_SERIAL.print(read_result.xel[1].error);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(F(", Param Length: "));DEBUG_SERIAL.print(read_result.xel[1].length);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();  
  delay(100);
}


