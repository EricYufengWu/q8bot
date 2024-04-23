#include <Arduino.h>
#include <WiFi.h>
#include <Dynamixel2Arduino.h>
#include <ESPNowW.h>
#include "q8Dynamixel.h"

/*==================================*/
/*========== Definitions ===========*/
/*==================================*/

// ESPNow
char myData[100];
bool incoming = false;

HardwareSerial mySerial0(0);
// // Dynamixel
// const uint8_t DXL_ID_CNT = 8;
// const uint8_t DXL[DXL_ID_CNT] = {11, 12, 13, 14, 15, 16, 17, 18};
// const uint8_t DXL_DIR_PIN = 8;
// const float DXL_PROTOCOL_VERSION = 2.0;
// const uint16_t user_pkt_buf_cap = 128;
// uint8_t user_pkt_buf[user_pkt_buf_cap];

// // Struct definitions for br (bulk read) and bw (bulk write)
// struct br_data_xel{
//   int32_t present_position;
// } __attribute__((packed));
// struct bw_data_xel{
//   int32_t goal_position;
// } __attribute__((packed));

// struct br_data_xel br_data_xel[DXL_ID_CNT];
// DYNAMIXEL::InfoBulkReadInst_t br_infos;
// DYNAMIXEL::XELInfoBulkRead_t info_xels_br[DXL_ID_CNT];

// struct bw_data_xel bw_data_xel[DXL_ID_CNT];
// DYNAMIXEL::InfoBulkWriteInst_t bw_infos;
// DYNAMIXEL::XELInfoBulkWrite_t info_xels_bw[DXL_ID_CNT];

// #define DEBUG_SERIAL Serial
// HardwareSerial MySerial0(0);
// Dynamixel2Arduino dxl(MySerial0, DXL_DIR_PIN);
// using namespace ControlTableItem; // Required for Dynamixel

// /*==================================*/
// /*======== Custom Functions ========*/
// /*==================================*/


// void enableTorque(){
//   for (int i = 0; i < DXL_ID_CNT; i++){
//     dxl.torqueOn(DXL[i]);
//   }
// }

// void disableTorque(){
//   for (int i = 0; i < DXL_ID_CNT; i++){
//     dxl.torqueOff(DXL[i]);
//   }
// }

// void setOperatingMode(uint8_t MODE = OP_EXTENDED_POSITION){
//   // Set the correct operating mode for all Dynamixels
//   for (int i = 0; i < DXL_ID_CNT; i++){
//     dxl.torqueOff(DXL[i]);
//     dxl.setOperatingMode(DXL[i], MODE);
//     dxl.torqueOn(DXL[i]);
//   }
// }

void onRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  // ESPNow callback function to receive data
  // char macStr[18];
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4],
  //          mac_addr[5]);
  // Serial.print("Last Packet Recv from: ");
  // Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Received a packet with size of: ");
  // Serial.println(len);
  Serial.println(myData);
}

/*==================================*/
/*=========== Main Code ============*/
/*==================================*/

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(100);
  }
  Serial.println("ESPNow receiver Demo");
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();
  ESPNow.init();
  ESPNow.reg_recv_cb(onRecv);

  // q8.begin();

  // dxl.begin(1000000);
  // setOperatingMode();
  
  // // fill the members of structure for bulkRead using external user packet buffer
  // br_infos.packet.p_buf = user_pkt_buf;
  // br_infos.packet.buf_capacity = user_pkt_buf_cap;
  // br_infos.packet.is_completed = false;
  // br_infos.p_xels = info_xels_br;
  // br_infos.xel_count = 0;

  // for (int i = 0; i < DXL_ID_CNT; i++){
  //   info_xels_br[i].id = DXL[i];
  //   info_xels_br[i].addr = 132; // Present Position of X serise.
  //   info_xels_br[i].addr_length = 4; // Present Current + Position + Velocity
  //   info_xels_br[i].p_recv_buf = reinterpret_cast<uint8_t*>(&br_data_xel[i]);
  //   br_infos.xel_count++;
  // }
  // br_infos.is_info_changed = true;

  // // Fill the members of structure for bulkWrite using internal packet buffer
  // bw_infos.packet.p_buf = nullptr;
  // bw_infos.packet.is_completed = false;
  // bw_infos.p_xels = info_xels_bw;
  // bw_infos.xel_count = 0;

  // for (int i = 0; i < DXL_ID_CNT; i++){
  //   bw_data_xel[i].goal_position = 0;
  //   info_xels_bw[i].id = DXL[i];
  //   info_xels_bw[i].addr = 116; // Goal Position of X serise.
  //   info_xels_bw[i].addr_length = 4; // Goal Position
  //   info_xels_bw[i].p_data = reinterpret_cast<uint8_t*>(&bw_data_xel[i]);
  //   bw_infos.xel_count++;
  // }
  // bw_infos.is_info_changed = true;

  // // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  // for (int i = 0; i < DXL_ID_CNT; i++){
  //   dxl.writeControlTableItem(PROFILE_VELOCITY, DXL[i], 1000);
  //   dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL[i], 300);
  // }
}

void loop() {
  // put your main code here, to run repeatedly:

  // q8.moveAll(512);
  Serial.println("Hello");
  delay(1000);

  // uint8_t recv_cnt;
  
  // for (int i = 0; i < DXL_ID_CNT; i++){
  //   bw_data_xel[i].goal_position+=512;
  //   if(bw_data_xel[i].goal_position >= 1023){
  //     bw_data_xel[i].goal_position = 0;
  //   }
  // }
  // bw_infos.is_info_changed = true;

  // dxl.bulkWrite(&bw_infos);
  // delay(1000);

  // // Bulk Write
  // if(dxl.bulkWrite(&bw_infos) == true){
  //   DEBUG_SERIAL.println("[BulkWrite] Success");
  //   for (int i = 0; i < DXL_ID_CNT; i++){
  //     DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(bw_infos.p_xels[i].id);
  //     DEBUG_SERIAL.print("\t Goal Position: ");DEBUG_SERIAL.println(bw_data_xel[i].goal_position);
  //   }
  // } else{
  //   DEBUG_SERIAL.print("[BulkWrite] Fail, Lib error code: ");
  //   DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  // }
  // delay(1000);

  // // Bulk Read
  // recv_cnt = dxl.bulkRead(&br_infos);
  // if(recv_cnt > 0){
  //   DEBUG_SERIAL.println("[BulkRead] Success");
  //   for (int i = 0; i < DXL_ID_CNT; i++){
  //     DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(br_infos.p_xels[i].id);
  //     DEBUG_SERIAL.print("\t Present Position: ");DEBUG_SERIAL.println(br_data_xel[i].present_position);
  //   }
  // } else{
  //   DEBUG_SERIAL.print("[BulkRead] Fail, Lib error code: ");
  //   DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  // }
  // delay(1000);
}