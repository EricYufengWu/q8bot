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
 

/* bulkRead
  Structures containing the necessary information to process the 'bulkRead' packet.

  typedef struct XELInfoBulkRead{
    uint16_t addr;
    uint16_t addr_length;
    uint8_t *p_recv_buf;
    uint8_t id;
    uint8_t error;
  } __attribute__((packed)) XELInfoBulkRead_t;

  typedef struct InfoBulkReadInst{
    XELInfoBulkRead_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed)) InfoBulkReadInst_t;
*/

/* bulkWrite
  Structures containing the necessary information to process the 'bulkWrite' packet.

  typedef struct XELInfoBulkWrite{
    uint16_t addr;
    uint16_t addr_length;
    uint8_t* p_data;
    uint8_t id;
  } __attribute__((packed)) XELInfoBulkWrite_t;

  typedef struct InfoBulkWriteInst{
    XELInfoBulkWrite_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed)) InfoBulkWriteInst_t;
*/

const uint8_t DXL_ID_CNT = 8;
const uint8_t DXL[DXL_ID_CNT] = {11, 12, 13, 14, 15, 16, 17, 18};
const uint8_t DXL_DIR_PIN = 8;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Define some struct structures for br (bulk read) and bw (bulk write)
struct br_data_xel{
  int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
} __attribute__((packed));
struct bw_data_xel{
  int32_t goal_position;
} __attribute__((packed));

struct br_data_xel br_data_xel[DXL_ID_CNT];
// struct br_data_xel br_data_xel_2;
DYNAMIXEL::InfoBulkReadInst_t br_infos;
DYNAMIXEL::XELInfoBulkRead_t info_xels_br[DXL_ID_CNT];

struct bw_data_xel bw_data_xel[DXL_ID_CNT];
// struct bw_data_xel bw_data_xel_2;
DYNAMIXEL::InfoBulkWriteInst_t bw_infos;
DYNAMIXEL::XELInfoBulkWrite_t info_xels_bw[DXL_ID_CNT];

#define DEBUG_SERIAL Serial
HardwareSerial MySerial0(0);
Dynamixel2Arduino dxl(MySerial0, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  
  for (int i = 0; i < DXL_ID_CNT; i++){
    dxl.torqueOff(DXL[i]);
    dxl.setOperatingMode(DXL[i], OP_EXTENDED_POSITION);
    dxl.torqueOn(DXL[i]);
  }
  
  // fill the members of structure for bulkRead using external user packet buffer
  br_infos.packet.p_buf = user_pkt_buf;
  br_infos.packet.buf_capacity = user_pkt_buf_cap;
  br_infos.packet.is_completed = false;
  br_infos.p_xels = info_xels_br;
  br_infos.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT; i++){
    info_xels_br[i].id = DXL[i];
    info_xels_br[i].addr = 126; // Present Position of X serise.
    info_xels_br[i].addr_length = 2+4+4; // Present Current + Position + Velocity
    info_xels_br[i].p_recv_buf = reinterpret_cast<uint8_t*>(&br_data_xel[i]);
    br_infos.xel_count++;
  }
  br_infos.is_info_changed = true;

  // Fill the members of structure for bulkWrite using internal packet buffer
  bw_infos.packet.p_buf = nullptr;
  bw_infos.packet.is_completed = false;
  bw_infos.p_xels = info_xels_bw;
  bw_infos.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT; i++){
    bw_data_xel[i].goal_position = 0;
    info_xels_bw[i].id = DXL[i];
    info_xels_bw[i].addr = 116; // Goal Position of X serise.
    info_xels_bw[i].addr_length = 4; // Goal Velocity
    info_xels_bw[i].p_data = reinterpret_cast<uint8_t*>(&bw_data_xel[i]);
    bw_infos.xel_count++;
  }
  bw_infos.is_info_changed = true;

  // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  for (int i = 0; i < DXL_ID_CNT; i++){
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL[i], 1000);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL[i], 300);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t try_count = 0;
  uint8_t recv_cnt;
  
  for (int i = 0; i < DXL_ID_CNT; i++){
    bw_data_xel[i].goal_position+=1024;
    if(bw_data_xel[i].goal_position >= 4095){
      bw_data_xel[i].goal_position = 0;
    }
  }
  bw_infos.is_info_changed = true;

  // Bulk Write
  DEBUG_SERIAL.print("\n>>>>>> Bulk Instruction Test : ");
  DEBUG_SERIAL.println(try_count++);
  if(dxl.bulkWrite(&bw_infos) == true){
    DEBUG_SERIAL.println("[BulkWrite] Success");
    for (int i = 0; i < DXL_ID_CNT; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(bw_infos.p_xels[i].id);
      DEBUG_SERIAL.print("\t Goal Position: ");DEBUG_SERIAL.println(bw_data_xel[i].goal_position);
    }
  } else{
    DEBUG_SERIAL.print("[BulkWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  
  delay(1000);

  // Bulk Read
  recv_cnt = dxl.bulkRead(&br_infos);
  if(recv_cnt > 0){
    DEBUG_SERIAL.print("[BulkRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);
    for (int i = 0; i < DXL_ID_CNT; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(br_infos.p_xels[i].id);
      // DEBUG_SERIAL.print(", Error: ");DEBUG_SERIAL.println(br_infos.p_xels[i].error);
      // DEBUG_SERIAL.print("\t Present Current: ");DEBUG_SERIAL.println(br_data_xel_1.present_current);
      // DEBUG_SERIAL.print("\t Present Velocity: ");DEBUG_SERIAL.println(br_data_xel_1.present_velocity);
      DEBUG_SERIAL.print("\t Present Position: ");DEBUG_SERIAL.println(br_data_xel[i].present_position);
    }
  } else{
    DEBUG_SERIAL.print("[BulkRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println("=======================================================");

  delay(1000);
}