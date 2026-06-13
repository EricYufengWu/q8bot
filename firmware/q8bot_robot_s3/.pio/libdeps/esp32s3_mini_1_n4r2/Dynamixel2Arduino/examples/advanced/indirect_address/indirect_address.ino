// Copyright 2022 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Example Environment
//
// - DYNAMIXEL: X series
//              ID = 1 & 2, Baudrate = 57600bps, DYNAMIXEL Protocol 2.0
// - Controller: Arduino MKR ZERO
//               DYNAMIXEL Shield for Arduino MKR
// - https://emanual.robotis.com/docs/en/parts/interface/mkr_shield/
//
// Author: Ashe Kim

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


const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;

const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};

const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint16_t INDIRECT_ADDR_NUM = 5;
uint16_t INDIRECT_ADDR_ARRY[INDIRECT_ADDR_NUM] = {65, 116, 117, 118, 119};

const uint16_t ID_ADDR_LEN = INDIRECT_ADDR_NUM * 2; // Data Length 2*INDIRECT_ADDR_NUM, Can differ depending on how many address to access. 
const uint16_t ID_START_ADDR = 168; // Indirect Address1 Address. Starting Data Addr, Can differ Depending on what address to access

const uint16_t SW_ADDR_LEN = 5; // Data Length (1+4), Can differ depending on how many address to access. 
const uint16_t SW_START_ADDR = 224; // Indirect Data1 Address. Starting Data Addr, Can differ Depending on what address to access

const uint16_t SR_ADDR_LEN = 4; // Data Length 4, Can differ depending on how many address to access. 
const uint16_t SR_START_ADDR = 132; // Present Position Address. Starting Data Addr, Can differ Depending on what address to access

typedef struct id_data{
  uint8_t* indirect_addr;
} __attribute__((packed)) id_data_t;

typedef struct sw_data{
  int8_t led;
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;

id_data_t id_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t id_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_id[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

int8_t led_state[2] = {0, 1};
int32_t position_state[2] = {1024, 1536};
uint8_t state_index = 0;

void setup() {
  // put your setup code here, to run once:
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);
  
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  ////////////////////////////////////////////////////////////////////////////////////
  // IndirectAddress                                                                //
  ////////////////////////////////////////////////////////////////////////////////////

  id_infos.packet.p_buf = nullptr;
  id_infos.packet.is_completed = false;
  id_infos.addr = ID_START_ADDR;
  id_infos.addr_length = ID_ADDR_LEN;
  id_infos.p_xels = info_xels_id;
  id_infos.xel_count = 0;

  id_data[0].indirect_addr = (uint8_t*)&INDIRECT_ADDR_ARRY;
  id_data[1].indirect_addr = (uint8_t*)&INDIRECT_ADDR_ARRY;

  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_id[i].id = DXL_ID_LIST[i];
    info_xels_id[i].p_data = id_data[i].indirect_addr;
    id_infos.xel_count++;
  }
  id_infos.is_info_changed = true;

  ////////////////////////////////////////////////////////////////////////////////////
  // Fill the members of structure to syncWrite using internal packet buffer        //
  ////////////////////////////////////////////////////////////////////////////////////

  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  sw_data[0].led = led_state[state_index];
  sw_data[1].led = led_state[state_index];  
  sw_data[0].goal_position = position_state[state_index];
  sw_data[1].goal_position = position_state[state_index];

  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].led;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;

  ////////////////////////////////////////////////////////////////////////////////////
  // Fill the members of structure to syncRead using external user packet buffer    //
  ////////////////////////////////////////////////////////////////////////////////////

  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;  

  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t try_count = 0;
  uint8_t i;
  uint8_t recv_cnt;

  delay(250);

  ////////////////////////////////////////////////////////////////////////////////////
  // setIndirectAddress                                                             //
  ////////////////////////////////////////////////////////////////////////////////////

  DEBUG_SERIAL.print("\n>>>>>>>>>>>>> Indirect Address Test : ");
  DEBUG_SERIAL.println(try_count++);
  if(dxl.syncWrite(&id_infos) == true){
    DEBUG_SERIAL.println("[Indirect Address] Success!!");
    for(i=0; i<id_infos.xel_count; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(id_infos.p_xels[i].id);
      DEBUG_SERIAL.print(", ADDR[0]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data));
      DEBUG_SERIAL.print("\t ADDR[1]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data+1));
      DEBUG_SERIAL.print("\t ADDR[2]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data+2));      
      DEBUG_SERIAL.print("\t ADDR[3]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data+3));
      DEBUG_SERIAL.print("\t ADDR[4]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data+4));
      DEBUG_SERIAL.print("\t ADDR[5]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data+5));
      DEBUG_SERIAL.print("\t ADDR[6]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data+6));
      DEBUG_SERIAL.print("\t ADDR[7]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data+7));
      DEBUG_SERIAL.print("\t ADDR[8]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data+8));
      DEBUG_SERIAL.print("\t ADDR[9]: ");DEBUG_SERIAL.println(*(info_xels_id[i].p_data+9));      
    }
  } else {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(250);

  ////////////////////////////////////////////////////////////////////////////////////
  // syncWrite                                                                      //
  ////////////////////////////////////////////////////////////////////////////////////

  for(i = 0; i < DXL_ID_CNT; i++){
    sw_data[i].led = led_state[state_index];
    sw_data[i].goal_position = position_state[state_index];
  }
  sw_infos.is_info_changed = true;

  if(dxl.syncWrite(&sw_infos) == true){
    DEBUG_SERIAL.println("[SyncWrite] Success!!");
    for(i=0; i<sw_infos.xel_count; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print(", LED: ");DEBUG_SERIAL.println(sw_data[i].led);
      DEBUG_SERIAL.print("\t Goal Position: ");DEBUG_SERIAL.println(sw_data[i].goal_position);
    }
    if(state_index == 0)
      state_index = 1;
    else
      state_index = 0;
  }else{
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(500);

  ////////////////////////////////////////////////////////////////////////////////////
  // syncRead                                                                       //
  ////////////////////////////////////////////////////////////////////////////////////

  recv_cnt = dxl.syncRead(&sr_infos);
  if(recv_cnt > 0){
    DEBUG_SERIAL.print("[SyncRead] Success!! Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);
    for(i=0; i<recv_cnt; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
      DEBUG_SERIAL.print(", Error: ");DEBUG_SERIAL.println(sr_infos.p_xels[i].error);
      DEBUG_SERIAL.print("\t Present Position: ");DEBUG_SERIAL.println(sr_data[i].present_position);
    }
  }else{
    DEBUG_SERIAL.print("[SyncRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(250);
}
