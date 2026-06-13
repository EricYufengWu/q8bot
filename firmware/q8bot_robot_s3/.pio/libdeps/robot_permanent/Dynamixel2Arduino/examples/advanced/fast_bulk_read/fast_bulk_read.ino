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

// Author: Honghyun Kim

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


/* fastBulkRead
  DYNAMIXEL PROTOCOL 1.0 does NOT support Fast Bulk Read feature.
  Structures containing the necessary information to process the 'fastBulkRead' packet.

  struct XELInfoBulkRead_t {
    uint16_t addr;
    uint16_t addr_length;
    uint8_t *p_recv_buf;
    uint8_t id;
    uint8_t error;
  } __attribute__((packed));

  struct InfoBulkReadInst_t {
    XELInfoBulkRead_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed));
*/

/* bulkWrite
  DYNAMIXEL PROTOCOL 1.0 does NOT support Bulk Write feature.
  Structures containing the necessary information to process the 'bulkWrite' packet.

  struct XELInfoBulkWrite_t {
    uint16_t addr;
    uint16_t addr_length;
    uint8_t* p_data;
    uint8_t id;
  } __attribute__((packed));

  struct InfoBulkWriteInst_t {
    XELInfoBulkWrite_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed));
*/


const uint8_t DXL_1_ID = 1;
const uint8_t DXL_2_ID = 2;
const uint8_t DXL_ID_CNT = 2;
const uint16_t BUF_SIZE = 128; 
uint8_t user_pkt_buf[BUF_SIZE];

struct BrDataXel1 {
  int16_t present_current;
  int32_t present_velocity;
} __attribute__((packed));

struct BrDataXel2 {
  int32_t present_position;
} __attribute__((packed));

struct BwDataXel1 {
  int32_t goal_velocity;
} __attribute__((packed));

struct BwDataXel2 {
  int32_t goal_position;
} __attribute__((packed));


BrDataXel1 br_data_xel_1;
BrDataXel2 br_data_xel_2;
DYNAMIXEL::InfoBulkReadInst_t br_infos;
DYNAMIXEL::XELInfoBulkRead_t info_xels_br[DXL_ID_CNT];

BwDataXel1 bw_data_xel_1;
BwDataXel2 bw_data_xel_2;
DYNAMIXEL::InfoBulkWriteInst_t bw_infos;
DYNAMIXEL::XELInfoBulkWrite_t info_xels_bw[DXL_ID_CNT];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);

  dxl.torqueOff(DXL_1_ID);
  dxl.torqueOff(DXL_2_ID);
  dxl.setOperatingMode(DXL_1_ID, OP_VELOCITY);
  dxl.setOperatingMode(DXL_2_ID, OP_POSITION);
  dxl.torqueOn(DXL_1_ID);
  dxl.torqueOn(DXL_2_ID);

  // Fill the members of structure to fastSyncRead using external user packet buffer
  br_infos.packet.p_buf = user_pkt_buf;
  br_infos.packet.buf_capacity = BUF_SIZE;
  br_infos.packet.is_completed = false;
  br_infos.p_xels = info_xels_br;

  info_xels_br[0].id = DXL_1_ID;
  info_xels_br[0].addr = 126; // Present Current of X serise.
  info_xels_br[0].addr_length = 2 + 4; // Present Current + Present Velocity
  info_xels_br[0].p_recv_buf = (uint8_t*)&br_data_xel_1;

  info_xels_br[1].id = DXL_2_ID;
  info_xels_br[1].addr = 132; // Present Position of X serise.
  info_xels_br[1].addr_length = 4; // Present Position + Present Velocity
  info_xels_br[1].p_recv_buf = (uint8_t*)&br_data_xel_2;

  br_infos.xel_count = DXL_ID_CNT;
  br_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  bw_infos.packet.p_buf = nullptr;
  bw_infos.packet.is_completed = false;
  bw_infos.p_xels = info_xels_bw;

  bw_data_xel_1.goal_velocity = 0;
  info_xels_bw[0].id = DXL_1_ID;
  info_xels_bw[0].addr = 104; // Goal Velocity of X serise.
  info_xels_bw[0].addr_length = 4; // Goal Velocity
  info_xels_bw[0].p_data = (uint8_t*)&bw_data_xel_1;

  bw_data_xel_2.goal_position = 0;
  info_xels_bw[1].id = DXL_2_ID;
  info_xels_bw[1].addr = 116; // Goal Position of X serise.
  info_xels_bw[1].addr_length = 4; // Goal Position
  info_xels_bw[1].p_data = (uint8_t*)&bw_data_xel_2;

  bw_infos.xel_count = DXL_ID_CNT;
  bw_infos.is_info_changed = true;
}

void loop() {
  static uint32_t try_count = 0;

  bw_data_xel_1.goal_velocity += 5;
  if (bw_data_xel_1.goal_velocity >= 200)
    bw_data_xel_1.goal_velocity = 0;
  bw_data_xel_2.goal_position += 255;
  if (bw_data_xel_2.goal_position >= 1023)
    bw_data_xel_2.goal_position = 0;
  bw_infos.is_info_changed = true;

  DEBUG_SERIAL.print("\n>>>>>> Bulk Instruction Test : ");
  DEBUG_SERIAL.println(try_count++);
  if (true == dxl.bulkWrite(&bw_infos)) {
    DEBUG_SERIAL.println("[BulkWrite] Success");
    DEBUG_SERIAL.print("  ID: ");
    DEBUG_SERIAL.println(bw_infos.p_xels[0].id);
    DEBUG_SERIAL.print("\t Goal Velocity: ");
    DEBUG_SERIAL.println(bw_data_xel_1.goal_velocity);
    DEBUG_SERIAL.print("  ID: ");
    DEBUG_SERIAL.println(bw_infos.p_xels[1].id);
    DEBUG_SERIAL.print("\t Goal Position: ");
    DEBUG_SERIAL.println(bw_data_xel_2.goal_position);
  } else {
    DEBUG_SERIAL.print("[BulkWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(250);

  // Transmit predefined fastBulkRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  uint8_t recv_cnt = dxl.fastBulkRead(&br_infos);
  if (recv_cnt > 0) {
    DEBUG_SERIAL.print("[fastBulkRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);

    DEBUG_SERIAL.print("  ID: ");
    DEBUG_SERIAL.print(br_infos.p_xels[0].id);
    DEBUG_SERIAL.print(", Error: ");
    DEBUG_SERIAL.println(br_infos.p_xels[0].error);
    DEBUG_SERIAL.print("\t Present Current: ");
    DEBUG_SERIAL.println(br_data_xel_1.present_current);
    DEBUG_SERIAL.print("\t Present Velocity: ");
    DEBUG_SERIAL.println(br_data_xel_1.present_velocity);

    DEBUG_SERIAL.print("  ID: ");
    DEBUG_SERIAL.print(br_infos.p_xels[1].id);
    DEBUG_SERIAL.print(", Error: ");
    DEBUG_SERIAL.println(br_infos.p_xels[1].error);    
    DEBUG_SERIAL.print("\t Present Position: ");
    DEBUG_SERIAL.println(br_data_xel_2.present_position);
  } else {
    DEBUG_SERIAL.print("[fastBulkRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(750);
}
