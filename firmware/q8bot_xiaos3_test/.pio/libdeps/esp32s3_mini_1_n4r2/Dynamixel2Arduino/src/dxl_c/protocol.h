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
#ifndef DYNAMIXEL_PROTOCOL_H_
#define DYNAMIXEL_PROTOCOL_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

const int DXL_BROADCAST_ID = 0xFE;

namespace DYNAMIXEL {

struct InfoFromPing_t {
  uint8_t id;
  uint16_t model_number;
  uint8_t firmware_version;
};

struct InfoSyncBulkBuffer_t {
  uint8_t* p_buf;
  uint16_t buf_capacity;
  uint16_t gen_length;
  bool is_completed;
} __attribute__((packed));

/* Sync Instructions */
struct XELInfoSyncRead_t {
  uint8_t *p_recv_buf;
  uint8_t id;
  uint8_t error;
} __attribute__((packed));

struct InfoSyncReadInst_t {
  uint16_t addr;
  uint16_t addr_length;
  XELInfoSyncRead_t* p_xels;
  uint8_t xel_count;
  bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
} __attribute__((packed));

struct XELInfoSyncWrite_t {
  uint8_t* p_data;
  uint8_t id;
} __attribute__((packed));

struct InfoSyncWriteInst_t {
  uint16_t addr;
  uint16_t addr_length;
  XELInfoSyncWrite_t* p_xels;
  uint8_t xel_count;
  bool is_info_changed;
  InfoSyncBulkBuffer_t packet;
} __attribute__((packed));

/* Bulk Instructions */
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

} // namespace DYNAMIXEL

#ifdef __cplusplus
extern "C" {
#endif

// http://emanual.robotis.com/docs/en/dxl/protocol1/#instruction
// http://emanual.robotis.com/docs/en/dxl/protocol2/#instruction
enum DXLInstruction{
  DXL_INST_PING = 0x01,
  DXL_INST_READ = 0x02,
  DXL_INST_WRITE = 0x03,
  DXL_INST_REG_WRITE = 0x04,
  DXL_INST_ACTION = 0x05,
  DXL_INST_FACTORY_RESET = 0x06,
  DXL_INST_REBOOT = 0x08,
  DXL_INST_CLEAR = 0x10, //ONLY Protocol2.0
  DXL_INST_STATUS = 0x55, //ONLY Protocol2.0
  DXL_INST_SYNC_READ = 0x82, //ONLY Protocol2.0
  DXL_INST_FAST_SYNC_READ = 0x8A, //ONLY Protocol2.0
  DXL_INST_SYNC_WRITE = 0x83,
  DXL_INST_BULK_READ = 0x92,
  DXL_INST_FAST_BULK_READ = 0x9A, // ONLY Protocol2.0
  DXL_INST_BULK_WRITE = 0x93 //ONLY Protocol2.0
};

enum DXL1_0PacketError{
  DXL1_0_ERR_INPUT_VOLTAGE_BIT = 0,
  DXL1_0_ERR_ANGLE_LIMIT_BIT,
  DXL1_0_ERR_OVERHEATING_BIT,
  DXL1_0_ERR_RANGE_BIT,
  DXL1_0_ERR_CHECKSUM_BIT,
  DXL1_0_ERR_OVERLOAD_BIT,
  DXL1_0_ERR_INSTRUCTION_BIT
};

enum DXL2_0PacketError{
  DXL2_0_ERR_NONE = 0x00,
  DXL2_0_ERR_RESULT_FAIL,
  DXL2_0_ERR_INST_ERROR,
  DXL2_0_ERR_CRC_ERROR,
  DXL2_0_ERR_DATA_RANGE,
  DXL2_0_ERR_DATA_LENGTH,
  DXL2_0_ERR_DATA_LIMIT,
  DXL2_0_ERR_ACCESS
};

// To support legacy
#define DXL_ERR_ACCESS DXL2_0_ERR_ACCESS

enum DXL1_0PacketState{
  DXL1_0_PACKET_PARSING_STATE_IDLE = 0,
  DXL1_0_PACKET_PARSING_STATE_ID,
  DXL1_0_PACKET_PARSING_STATE_LENGTH,
  DXL1_0_PACKET_PARSING_STATE_INST_ERR,
  DXL1_0_PACKET_PARSING_STATE_PARAM,
  DXL1_0_PACKET_PARSING_STATE_CHECK_SUM
};

enum DXL2_0PacketState{
  DXL2_0_PACKET_PARSING_STATE_IDLE = 0,
  DXL2_0_PACKET_PARSING_STATE_RESERVED,
  DXL2_0_PACKET_PARSING_STATE_ID,
  DXL2_0_PACKET_PARSING_STATE_LENGTH_L,
  DXL2_0_PACKET_PARSING_STATE_LENGTH_H,
  DXL2_0_PACKET_PARSING_STATE_INST,
  DXL2_0_PACKET_PARSING_STATE_ERROR,
  DXL2_0_PACKET_PARSING_STATE_PARAM,
  DXL2_0_PACKET_PARSING_STATE_CRC_L,
  DXL2_0_PACKET_PARSING_STATE_CRC_H
};

enum DXLLibErrorCode
{
  DXL_LIB_OK = 0,
  DXL_LIB_PROCEEDING,

  DXL_LIB_ERROR_NOT_SUPPORTED,
  DXL_LIB_ERROR_TIMEOUT,
  DXL_LIB_ERROR_INVAILD_ID,
  DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST,
  DXL_LIB_ERROR_NULLPTR,
  DXL_LIB_ERROR_LENGTH,
  DXL_LIB_ERROR_INVAILD_ADDR,
  DXL_LIB_ERROR_ADDR_LENGTH,
  DXL_LIB_ERROR_BUFFER_OVERFLOW,
  DXL_LIB_ERROR_PORT_NOT_OPEN,
  DXL_LIB_ERROR_WRONG_PACKET,
  DXL_LIB_ERROR_CHECK_SUM,
  DXL_LIB_ERROR_CRC,
  DXL_LIB_ERROR_INVAILD_DATA_LENGTH,
  DXL_LIB_ERROR_MEMORY_ALLOCATION_FAIL,
  DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION,
  DXL_LIB_ERROR_NOT_INITIALIZED,
  DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE,
  DXL_LIB_ERROR_PORT_WRITE
};

enum DXL_FastParamState {
  DXL_FAST_PARAM_ERROR = 0,
  DXL_FAST_PARAM_ID,
  DXL_FAST_PARAM_DATA,
  DXL_FAST_PARAM_CRC_L,
  DXL_FAST_PARAM_CRC_H
};

typedef struct InfoToParseDXLPacket{
  uint8_t header[3];
  uint8_t header_cnt;
  uint8_t id;
  uint8_t protocol_ver;
  uint8_t inst_idx;
  uint8_t err_idx;
  uint8_t *p_param_buf;
  uint16_t param_buf_capacity;
  uint16_t recv_param_len;
  uint16_t packet_len;
  uint16_t calculated_crc;
  uint16_t recv_crc;
  uint8_t calculated_check_sum;
  uint8_t recv_check_sum;
  uint8_t reserved;
  uint8_t parse_state;
  uint8_t xel_count;
  bool is_init;
  uint8_t xel_index;
  uint16_t buf_index;
  uint16_t param_length;
  uint8_t fast_param_state;
}InfoToParseDXLPacket_t;

typedef struct InfoToMakeDXLPacket{
  uint8_t id;
  uint8_t protocol_ver;
  uint8_t inst_idx;
  uint8_t err_idx;
  uint8_t *p_packet_buf;
  uint16_t packet_buf_capacity;
  uint16_t param_length;
  uint16_t generated_packet_length;
  bool is_init;
}InfoToMakeDXLPacket_t;

typedef int32_t DXLLibErrorCode_t;

DXLLibErrorCode_t begin_make_dxl_packet(InfoToMakeDXLPacket_t* p_make_packet, 
  uint8_t id, uint8_t protocol_ver, uint8_t inst_idx, uint8_t err_idx, 
  uint8_t* p_packet_buf, uint16_t packet_buf_capacity);
DXLLibErrorCode_t add_param_to_dxl_packet(InfoToMakeDXLPacket_t* p_make_packet,
  uint8_t *p_param, uint16_t param_len);
DXLLibErrorCode_t end_make_dxl_packet(InfoToMakeDXLPacket_t* p_make_packet);

DXLLibErrorCode_t begin_parse_dxl_packet(InfoToParseDXLPacket_t* p_parse_packet, 
  uint8_t protocol_ver, uint8_t* p_param_buf, uint16_t param_buf_cap);
DXLLibErrorCode_t parse_dxl_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data);

DXLLibErrorCode_t fast_begin_parse_dxl_packet(InfoToParseDXLPacket_t* p_parse_packet, 
  uint8_t protocol_ver);//, uint8_t* p_param_buf, uint16_t param_buf_cap, uint8_t xel_count);
DXLLibErrorCode_t fast_parse_dxl_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data,
                                        DYNAMIXEL::InfoSyncReadInst_t *sync_read, DYNAMIXEL::InfoBulkReadInst_t *bulk_read);

#ifdef __cplusplus
}
#endif

#endif /* DYNAMIXEL_PROTOCOL_H_ */
