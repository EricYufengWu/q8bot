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

#ifndef DYNAMIXEL_MASTER_HPP_
#define DYNAMIXEL_MASTER_HPP_


#include "dxl_c/protocol.h"
#include "port_handler.h"
#include "config.h"


const int UNREGISTERED_MODEL = (uint16_t)0xFFFF;
const int COMMON_MODEL_NUMBER_ADDR = 0;
const int COMMON_MODEL_NUMBER_ADDR_LENGTH = 2;


// >> Legacy (Deprecated since v0.4.0)
typedef struct InfoForSyncReadParam{
  uint8_t id;
} InfoForSyncReadParam_t;

typedef struct ParamForSyncReadInst{
  uint16_t addr;
  uint16_t length;
  uint8_t id_count;
  InfoForSyncReadParam_t xel[DXL_MAX_NODE]; //refer to below.
} ParamForSyncReadInst_t;

typedef struct XelInfoForSyncWriteParam{
  uint8_t id;
  uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
} XelInfoForSyncWriteParam_t;

typedef struct ParamForSyncWriteInst{
  uint16_t addr;
  uint16_t length;
  uint8_t id_count;
  XelInfoForSyncWriteParam_t xel[DXL_MAX_NODE]; //refer to below.
} ParamForSyncWriteInst_t;

typedef struct XelInfoForBulkReadParam{
  uint8_t id;
  uint16_t addr;
  uint16_t length;
} XelInfoForBulkReadParam_t;

typedef struct ParamForBulkReadInst{
  uint8_t id_count;
  XelInfoForBulkReadParam_t xel[DXL_MAX_NODE]; //refer to below.
} ParamForBulkReadInst_t;

typedef struct XelInfoForBulkWriteParam{
  uint8_t id;
  uint16_t addr;
  uint16_t length;
  uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
} XelInfoForBulkWriteParam_t;

typedef struct ParamForBulkWriteInst{
  uint8_t id_count;
  XelInfoForBulkWriteParam_t xel[DXL_MAX_NODE]; //refer to below. 
} ParamForBulkWriteInst_t;

typedef struct XelInfoForStatusInst{
  uint8_t id;
  uint16_t length;
  uint8_t error;
  uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
} XelInfoForStatusInst_t;

typedef struct RecvInfoFromStatusInst{
  uint8_t id_count;
  XelInfoForStatusInst_t xel[DXL_MAX_NODE]; //refer to below.
} RecvInfoFromStatusInst_t;
// << Legacy (Deprecated since v0.4.0)


namespace DYNAMIXEL {

class Master
{
  public:
    /**
     * @brief The constructor.
     * @code
     * const int DXL_DIR_PIN = 2;
     * const float PROTOCOL_VER = 2.0;
     * DYNAMIXEL::SerialPortHandler dxl_port(Serial1, DXL_DIR_PIN);
     * DYNAMIXEL::Master dxl_master(dxl_port, PROTOCOL_VER);
     * @endcode
     * @param port The DXLPortHandler instance you want to use on the board to communicate with DYNAMIXELs.
     *             It can be used not only for Serial but also for other communication port handlers like SerialPortHandler class.
     * @param protocol_ver DYNAMIXEL protocol version used for communications. (default : 2.0)
     */
    Master(DXLPortHandler &port, float protocol_ver = 2.0, uint16_t malloc_buf_size = 256);

    /**
     * @brief The constructor.
     *        This constructor must be added to the PortHanlder instance via the @setPort () function after creation.
     * @code
     * const float PROTOCOL_VER = 2.0;
     * DYNAMIXEL::Master dxl_master(PROTOCOL_VER);
     * @endcode
     * @param protocol_ver DYNAMIXEL protocol version used for communications. (default : 2.0)        
     */    
    Master(float protocol_ver = 2.0, uint16_t malloc_buf_size = 256);

    bool setPacketBuffer(uint8_t* p_buf, uint16_t buf_capacity);
    uint8_t* getPacketBuffer() const;
    uint16_t getPacketBufferCapacity() const;

    bool setPortProtocolVersion(float version);
    bool setPortProtocolVersionUsingIndex(uint8_t version_idx);
    float getPortProtocolVersion() const;

    bool setPort(DXLPortHandler &port);
    bool setPort(DXLPortHandler *p_port);
    DXLPortHandler* getPort() const;

    /* Instructions */
    uint8_t ping(uint8_t id, uint8_t *p_recv_id_array, uint8_t recv_array_capacity,
      uint32_t timeout_ms = 10);
    uint8_t ping(uint8_t id, InfoFromPing_t *recv_ping_info_array, uint8_t recv_array_cnt,
      uint32_t timeout_ms = 10);
    int32_t read(uint8_t id, uint16_t addr, uint16_t addr_length,
      uint8_t *p_recv_buf, uint16_t recv_buf_capacity, uint32_t timeout_ms = 10);
    bool write(uint8_t id, uint16_t addr, 
      const uint8_t *p_data, uint16_t data_length, uint32_t timeout_ms = 10);      
    bool writeNoResp(uint8_t id, uint16_t addr, 
      const uint8_t *p_data, uint16_t data_length);
    bool regWrite(uint8_t id, uint16_t addr, 
      const uint8_t *p_data, uint16_t data_length, uint32_t timeout_ms = 10);
    bool action(uint8_t id, uint32_t timeout_ms = 10);      
    bool factoryReset(uint8_t id, uint8_t option, uint32_t timeout_ms = 10);
    bool reboot(uint8_t id, uint32_t timeout_ms = 10);
    bool clear(uint8_t id, uint8_t option, uint32_t ex_option, uint32_t timeout_ms = 10);
    uint8_t syncRead(InfoSyncReadInst_t* p_info, uint32_t timeout_ms = 10);
    bool syncWrite(InfoSyncWriteInst_t* p_info);
    uint8_t bulkRead(InfoBulkReadInst_t* p_info, uint32_t timeout_ms = 10);
    bool bulkWrite(InfoBulkWriteInst_t* p_info);

    uint8_t fastSyncRead(InfoSyncReadInst_t* p_info, uint32_t timeout_ms = 10);
    uint8_t fastBulkRead(InfoBulkReadInst_t* p_info, uint32_t timeout_ms = 10);

    uint8_t getLastStatusPacketError() const;
    DXLLibErrorCode_t getLastLibErrCode() const;

    // >> Legacy (Deprecated since v0.4.0)
    bool syncRead(const ParamForSyncReadInst_t &param_info, RecvInfoFromStatusInst_t &recv_info, uint32_t timeout_ms = 100);
    bool syncWrite(const ParamForSyncWriteInst_t &param_info);
    bool bulkRead(const ParamForBulkReadInst_t &param_info, RecvInfoFromStatusInst_t &recv_info, uint32_t timeout_ms = 100);
    bool bulkWrite(const ParamForBulkWriteInst_t &param_info);
    // << Legacy (Deprecated since v0.4.0)

  protected:
    void setLastLibErrCode(DXLLibErrorCode_t err_code);

    // raw APIs
    bool txInstPacket(uint8_t id, uint8_t inst_idx, uint8_t *p_param, uint16_t param_len);
    const InfoToParseDXLPacket_t* rxStatusPacket(uint8_t* p_param_buf, uint16_t param_buf_cap, uint32_t timeout_ms = 10);
    const InfoToParseDXLPacket_t* fastRxStatusPacket(InfoSyncReadInst_t* sync_read, InfoBulkReadInst_t* bulk_read, uint32_t timeout_ms = 10);

  private:
    DXLPortHandler *p_port_;
    uint8_t protocol_ver_idx_;
    bool is_buf_malloced_;
    uint8_t *p_packet_buf_;
    uint16_t packet_buf_capacity_;
    InfoToMakeDXLPacket_t info_tx_packet_;
    InfoToParseDXLPacket_t info_rx_packet_;
    DXLLibErrorCode_t last_lib_err_;
};

} //namespace DYNAMIXEL


//legacy
typedef DYNAMIXEL::InfoFromPing_t XelInfoFromPing_t;


#endif /* DYNAMIXEL_MASTER_HPP_ */
