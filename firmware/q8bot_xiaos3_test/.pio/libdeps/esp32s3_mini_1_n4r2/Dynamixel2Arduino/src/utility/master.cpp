#include "master.h"


using namespace DYNAMIXEL;


Master::Master(DXLPortHandler &port, float protocol_ver, uint16_t malloc_buf_size)
: protocol_ver_idx_(2),
  is_buf_malloced_(false), packet_buf_capacity_(0),
  last_lib_err_(DXL_LIB_OK)
{
  setPort(port);
  setPortProtocolVersion(protocol_ver);
  
  if(malloc_buf_size > 0){
    p_packet_buf_ = new uint8_t[malloc_buf_size];
    if(p_packet_buf_ != nullptr){
      packet_buf_capacity_ = malloc_buf_size;
      is_buf_malloced_ = true;
    }
  }
  info_tx_packet_.is_init = false;
  info_rx_packet_.is_init = false;
}


Master::Master(float protocol_ver, uint16_t malloc_buf_size)
: protocol_ver_idx_(2),
  is_buf_malloced_(false), packet_buf_capacity_(0),
  last_lib_err_(DXL_LIB_OK)
{
  setPortProtocolVersion(protocol_ver);

  if(malloc_buf_size > 0){
    p_packet_buf_ = new uint8_t[malloc_buf_size];
    if(p_packet_buf_ != nullptr){
      packet_buf_capacity_ = malloc_buf_size;
      is_buf_malloced_ = true;
    }
  }
  info_tx_packet_.is_init = false;
  info_rx_packet_.is_init = false;  
}


bool 
Master::setPacketBuffer(uint8_t* p_buf, uint16_t buf_capacity)
{
  if(p_buf == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return false;
  }
  if(packet_buf_capacity_ == 0){
    last_lib_err_ = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
    return false;
  }

  if(is_buf_malloced_ == true){
    delete p_packet_buf_;
  }
  p_packet_buf_ = p_buf;
  packet_buf_capacity_ = buf_capacity;

  return true;
}


uint8_t* 
Master::getPacketBuffer() const
{
  return p_packet_buf_;
}


uint16_t 
Master::getPacketBufferCapacity() const
{
  return packet_buf_capacity_;
}


// Refer to http://emanual.robotis.com/#protocol
bool 
Master::setPortProtocolVersion(float version)
{
  uint8_t version_idx;

  if(version == 2.0){
    version_idx = 2;
  }else if(version == 1.0){
    version_idx = 1;
  }else{
    last_lib_err_ = DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;
    return false;
  }

  return setPortProtocolVersionUsingIndex(version_idx);
}


bool 
Master::setPortProtocolVersionUsingIndex(uint8_t version_idx)
{
  if(version_idx != 2 && version_idx != 1){
    return false;
  }
  protocol_ver_idx_ = version_idx;

  return true;
}


float 
Master::getPortProtocolVersion() const
{
  return (float)protocol_ver_idx_;
}


bool 
Master::setPort(DXLPortHandler *p_port)
{
  if(p_port == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return false;
  }

  p_port_ = p_port;

  return true;
}


bool 
Master::setPort(DXLPortHandler &port)
{
  p_port_ = &port;

  return true;
}


DXLPortHandler* 
Master::getPort() const
{
  return p_port_;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#ping
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#ping
uint8_t 
Master::ping(uint8_t id, uint8_t *p_recv_id_array, uint8_t recv_array_capacity, uint32_t timeout_ms)
{
  uint8_t ret_id_cnt = 0;
  uint32_t pre_time_ms;
  uint8_t rx_param[3];

  // Parameter exception handling
  if(p_recv_id_array == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return 0;
  }else if(recv_array_capacity == 0){
    last_lib_err_ = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
    return 0;
  }

  // Send Ping Instruction
  if(txInstPacket(id, DXL_INST_PING, nullptr, 0) == true){
    // Receive Status Packet
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket(rx_param, 3, timeout_ms) != nullptr){
        if(info_rx_packet_.id == id){
          p_recv_id_array[ret_id_cnt++] = info_rx_packet_.id;
        }
      }
    }else{
      timeout_ms = 3*253;
      pre_time_ms = millis();
      while(ret_id_cnt < recv_array_capacity)
      {
        if(rxStatusPacket(rx_param, 3, 3) != nullptr){
          p_recv_id_array[ret_id_cnt++] = info_rx_packet_.id;
        }

        if (millis()-pre_time_ms >= timeout_ms) {
          last_lib_err_ = DXL_LIB_ERROR_TIMEOUT;
          break;
        }
      }
    }
  }

  return ret_id_cnt;
}

uint8_t 
Master::ping(uint8_t id, InfoFromPing_t *recv_ping_info_array, uint8_t recv_array_cnt, uint32_t timeout_ms)
{
  uint8_t ret_id_cnt = 0;
  InfoFromPing_t *p_info = recv_ping_info_array;
  uint32_t pre_time_ms;

  // Parameter exception handling
  if(p_info == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return 0;
  }else if(recv_array_cnt == 0){
    last_lib_err_ = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
    return 0;
  }

  // Send Ping Instruction
  if(txInstPacket(id, DXL_INST_PING, nullptr, 0) == true){
    // Receive Status Packet
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket((uint8_t*)&p_info[ret_id_cnt].model_number, 3, timeout_ms) != nullptr){
        if(info_rx_packet_.id == id){
          p_info[ret_id_cnt++].id = info_rx_packet_.id;
        }
      }
    }else{
      timeout_ms = 3*253;
      pre_time_ms = millis();
      while(ret_id_cnt < recv_array_cnt)
      {
        if(rxStatusPacket((uint8_t*)&p_info[ret_id_cnt].model_number, 3, 3) != nullptr){
          p_info[ret_id_cnt++].id = info_rx_packet_.id;
        }

        if (millis()-pre_time_ms >= timeout_ms) {
          last_lib_err_ = DXL_LIB_ERROR_TIMEOUT;
          break;
        }
      }
    }
  }

  return ret_id_cnt;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#read
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#read
int32_t 
Master::read(uint8_t id, uint16_t addr, uint16_t addr_length,
 uint8_t *p_recv_buf, uint16_t recv_buf_capacity, uint32_t timeout_ms)
{
  int32_t ret_param_len = -1;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t param_len = 0;
  uint8_t tx_param[4];
  
  // Parameter exception handling
  if(p_recv_buf == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(addr_length == 0) {
    err = DXL_LIB_ERROR_ADDR_LENGTH;
  }else if(recv_buf_capacity < addr_length){
    err = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
  }else if (id == DXL_BROADCAST_ID) {
    err = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return -1;
  }
    
  // Send Read Instruction
  if(protocol_ver_idx_ == 2){
    tx_param[param_len++] = addr >> 0;
    tx_param[param_len++] = addr >> 8;
    tx_param[param_len++] = addr_length >> 0;
    tx_param[param_len++] = addr_length >> 8;    
  }else if(protocol_ver_idx_ == 1){
    tx_param[param_len++] = (uint8_t)addr&0xFF;
    tx_param[param_len++] = (uint8_t)addr_length&0xFF;
  }
  if(txInstPacket(id, DXL_INST_READ, (uint8_t*)tx_param, param_len) == true){
    if(rxStatusPacket(p_recv_buf, recv_buf_capacity, timeout_ms) != nullptr){
      ret_param_len = (int32_t)info_rx_packet_.recv_param_len;
    }
  }

  return ret_param_len;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#write
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#write
bool 
Master::write(uint8_t id, uint16_t addr, 
  const uint8_t *p_data, uint16_t data_length, uint32_t timeout_ms)
{
  bool ret = false;
  
  // Send Write Instruction
  if(writeNoResp(id, addr, p_data, data_length) == true){
    // Receive Status Packet
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
        if(protocol_ver_idx_ == 2){
          if(info_rx_packet_.err_idx == 0 || info_rx_packet_.err_idx == 0x80){
            ret = true;
          }
        }else if(protocol_ver_idx_ == 1){
          if(info_rx_packet_.err_idx == 0 
          || (info_rx_packet_.err_idx & (1<<DXL1_0_ERR_INPUT_VOLTAGE_BIT
                                        | 1<<DXL1_0_ERR_OVERHEATING_BIT 
                                        | 1<<DXL1_0_ERR_OVERLOAD_BIT)) != 0){
            ret = true;
          }
        }
      }
    }else{
      ret = true;
    }
  }

  return ret;
}


bool 
Master::writeNoResp(uint8_t id, uint16_t addr, const uint8_t *p_data, uint16_t data_length)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t param_len = 0;

  // Parameter exception handling
  if(p_data == nullptr || p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(data_length == 0) {
    err = DXL_LIB_ERROR_INVAILD_DATA_LENGTH;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  // Send Write Instruction
  if(protocol_ver_idx_ == 2){
    param_len = 2;
  }else if(protocol_ver_idx_ == 1){
    param_len = 1;
  }
  begin_make_dxl_packet(&info_tx_packet_, id, protocol_ver_idx_,
    DXL_INST_WRITE, 0, p_packet_buf_, packet_buf_capacity_);
  add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)&addr, param_len);
  param_len = data_length;
  add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)p_data, param_len);
  err = end_make_dxl_packet(&info_tx_packet_);
  if(err == DXL_LIB_OK){
    p_port_->write(info_tx_packet_.p_packet_buf, info_tx_packet_.generated_packet_length);
    ret = true;
  }

  last_lib_err_ = err;

  return ret;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#reg-write
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#reg-write
bool
Master::regWrite(uint8_t id, uint16_t addr, 
  const uint8_t *p_data, uint16_t data_length, uint32_t timeout_ms)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t param_len = 0;

  // Parameter exception handling
  if(p_data == nullptr || p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(data_length == 0) {
    err = DXL_LIB_ERROR_INVAILD_DATA_LENGTH;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  // Send Write Instruction
  if(protocol_ver_idx_ == 2){
    param_len = 2;
  }else if(protocol_ver_idx_ == 1){
    param_len = 1;
  }
  begin_make_dxl_packet(&info_tx_packet_, id, protocol_ver_idx_,
    DXL_INST_REG_WRITE, 0, p_packet_buf_, packet_buf_capacity_);
  add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)&addr, param_len);
  param_len = data_length;
  add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)p_data, param_len);
  err = end_make_dxl_packet(&info_tx_packet_);
  if(err == DXL_LIB_OK){
    p_port_->write(info_tx_packet_.p_packet_buf, info_tx_packet_.generated_packet_length);

    // Receive Status Packet
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
        if(protocol_ver_idx_ == 2){
          if(info_rx_packet_.err_idx == 0 || info_rx_packet_.err_idx == 0x80){
            ret = true;
          }
        }else if(protocol_ver_idx_ == 1){
          if(info_rx_packet_.err_idx == 0 
          || (info_rx_packet_.err_idx & (1<<DXL1_0_ERR_INPUT_VOLTAGE_BIT
                                        | 1<<DXL1_0_ERR_OVERHEATING_BIT 
                                        | 1<<DXL1_0_ERR_OVERLOAD_BIT)) != 0){
            ret = true;
          }
        }
      }
    }else{
      ret = true;
    }
  }

  last_lib_err_ = err;

  return ret;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#action
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#action
bool
Master::action(uint8_t id, uint32_t timeout_ms)
{
  bool ret = false;

  // Send Reboot Instruction
  if(txInstPacket(id, DXL_INST_ACTION, nullptr, 0) == true){
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
        if(protocol_ver_idx_ == 2){
          if(info_rx_packet_.err_idx == 0 || info_rx_packet_.err_idx == 0x80){
            ret = true;
          }
        }else if(protocol_ver_idx_ == 1){
          if(info_rx_packet_.err_idx == 0 
          || (info_rx_packet_.err_idx & (1<<DXL1_0_ERR_INPUT_VOLTAGE_BIT
                                        | 1<<DXL1_0_ERR_OVERHEATING_BIT 
                                        | 1<<DXL1_0_ERR_OVERLOAD_BIT)) != 0){
            ret = true;
          }
        }
      }
    }else{
      ret = true;
    }
  }

  return ret;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#factory-reset
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#factory-reset
bool 
Master::factoryReset(uint8_t id, uint8_t option, uint32_t timeout_ms)
{
  bool ret = false;
  uint8_t param_len = 0;
 
  // Send FactoryReset Instruction
  if(protocol_ver_idx_ == 2){
    param_len = 1;
  }
  if(txInstPacket(id, DXL_INST_FACTORY_RESET, (uint8_t*)&option, param_len) == true){
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
        if(protocol_ver_idx_ == 2){
          if(info_rx_packet_.err_idx == 0 || info_rx_packet_.err_idx == 0x80){
            ret = true;
          }
        }else if(protocol_ver_idx_ == 1){
          if(info_rx_packet_.err_idx == 0 
          || (info_rx_packet_.err_idx & (1<<DXL1_0_ERR_INPUT_VOLTAGE_BIT
                                        | 1<<DXL1_0_ERR_OVERHEATING_BIT 
                                        | 1<<DXL1_0_ERR_OVERLOAD_BIT)) != 0){
            ret = true;
          }
        }
      }
    }else{
      ret = true;
    }
  } 

  return ret;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#reboot
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#reboot
bool 
Master::reboot(uint8_t id, uint32_t timeout_ms)
{ 
  bool ret = false;

  // Send Reboot Instruction
  if(txInstPacket(id, DXL_INST_REBOOT, nullptr, 0) == true){
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
        if(protocol_ver_idx_ == 2){
          if(info_rx_packet_.err_idx == 0 || info_rx_packet_.err_idx == 0x80){
            ret = true;
          }
        }else if(protocol_ver_idx_ == 1){
          if(info_rx_packet_.err_idx == 0 
          || (info_rx_packet_.err_idx & (1<<DXL1_0_ERR_INPUT_VOLTAGE_BIT
                                        | 1<<DXL1_0_ERR_OVERHEATING_BIT 
                                        | 1<<DXL1_0_ERR_OVERLOAD_BIT)) != 0){
            ret = true;
          }
        }
      }
    }else{
      ret = true;
    }
  }

  return ret;
}


// (Protocol 1.0) Not Supported
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#clear
bool
Master::clear(uint8_t id, uint8_t option, uint32_t ex_option, uint32_t timeout_ms)
{
  bool ret = false;
  uint8_t tx_param[5];

  // Parameter exception handling
  if(protocol_ver_idx_ != 2){
    last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORTED;
    return false;
  }

  // http://emanual.robotis.com/docs/en/dxl/protocol2/#clear
  tx_param[0] = option;
  if(option == 0x01){ 
    tx_param[1] = 0x44;
    tx_param[2] = 0x58;
    tx_param[3] = 0x4C;
    tx_param[4] = 0x22;
  }else{
    memcpy(&tx_param[1], &ex_option, 4);
  }

  // Send Reboot Instruction
  if(txInstPacket(id, DXL_INST_CLEAR, tx_param, 5) == true){
    if(id != DXL_BROADCAST_ID){
      if(rxStatusPacket(nullptr, 0, timeout_ms) != nullptr){
        if(protocol_ver_idx_ == 2){
          if(info_rx_packet_.err_idx == 0 || info_rx_packet_.err_idx == 0x80){
            ret = true;
          }
        }
      }
    }else{
      ret = true;
    }
  }

  return ret;
}



// (Protocol 1.0) Not Supported
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#sync-read
uint8_t
Master::syncRead(InfoSyncReadInst_t* p_info, uint32_t timeout_ms)
{
  uint8_t i, recv_cnt = 0;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t tx_param[4], param_len = 0;
  uint8_t* p_packet_buf = nullptr;
  uint16_t packet_buf_cap;
  XELInfoSyncRead_t* p_xel;

  // Parameter exception handling
  if(p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(protocol_ver_idx_ != 2){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return 0;
  }

  if(p_info->packet.p_buf != nullptr){
    p_packet_buf = p_info->packet.p_buf;
    packet_buf_cap = p_info->packet.buf_capacity;
  }else{
    p_packet_buf = p_packet_buf_;
    packet_buf_cap = packet_buf_capacity_;
  }

  if(p_info->packet.p_buf == nullptr && info_tx_packet_.inst_idx != DXL_INST_SYNC_READ){
    p_info->packet.is_completed = false;
  }

  if(p_info->packet.is_completed == false || p_info->is_info_changed == true){
    err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_,
      DXL_INST_SYNC_READ, 0, p_packet_buf, packet_buf_cap);
    if(err == DXL_LIB_OK){
      tx_param[param_len++] = p_info->addr >> 0;
      tx_param[param_len++] = p_info->addr >> 8;
      tx_param[param_len++] = p_info->addr_length >> 0;
      tx_param[param_len++] = p_info->addr_length >> 8;
      err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
      if(err == DXL_LIB_OK){
        for(i=0; i<p_info->xel_count; i++){
          p_xel = &p_info->p_xels[i];
          err = add_param_to_dxl_packet(&info_tx_packet_, &p_xel->id, 1);
          if(err != DXL_LIB_OK){
            break;
          }
        }
        if(err == DXL_LIB_OK){
          err = end_make_dxl_packet(&info_tx_packet_);
          if(err == DXL_LIB_OK){
            p_info->packet.gen_length = info_tx_packet_.generated_packet_length;
            p_info->packet.is_completed = true;
            p_info->is_info_changed = false;
          }
        }
      }
    }
  }

  if(err == DXL_LIB_OK){
    p_port_->write(p_packet_buf, p_info->packet.gen_length);

    for(i=0; i<p_info->xel_count; i++)
    {
      p_xel = &p_info->p_xels[i];
      if(rxStatusPacket(p_xel->p_recv_buf, p_info->addr_length, timeout_ms) != nullptr){
        if(info_rx_packet_.id == p_xel->id){
          p_xel->error = info_rx_packet_.err_idx;
          recv_cnt++;
        }else{
          break;
        }
      }else{
        break;
      }
    }
  }else{
    p_info->packet.is_completed = false;
  }
  last_lib_err_ = err;

  return recv_cnt;
}

// (Protocol 1.0) Not Supported
// (Protocol 2.0) Refer to https://emanual.robotis.com/docs/en/dxl/protocol2/#fast-sync-read-0x8a
uint8_t Master::fastSyncRead(InfoSyncReadInst_t* p_info, uint32_t timeout_ms)
{
  uint8_t recv_cnt = 0;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t tx_param[4], param_len = 0;
  uint8_t* p_packet_buf = nullptr;
  uint16_t packet_buf_cap;
  XELInfoSyncRead_t* p_xel;

  // Parameter exception handling
  if (p_port_ == nullptr)
    err = DXL_LIB_ERROR_NULLPTR;
  else if (p_port_->getOpenState() != true)
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  else if (protocol_ver_idx_ != 2)
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  if (err != DXL_LIB_OK) {
    last_lib_err_ = err;
    return 0;
  }

  if (p_info->packet.p_buf != nullptr) {
    p_packet_buf = p_info->packet.p_buf;
    packet_buf_cap = p_info->packet.buf_capacity;
  } else {
    p_packet_buf = p_packet_buf_;
    packet_buf_cap = packet_buf_capacity_;
  }

  if ((p_info->packet.p_buf == nullptr) && (info_tx_packet_.inst_idx != DXL_INST_FAST_SYNC_READ))
    p_info->packet.is_completed = false;

  if ((p_info->packet.is_completed == false) || (p_info->is_info_changed == true)) {
    err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_, DXL_INST_FAST_SYNC_READ, 0, p_packet_buf, packet_buf_cap);
    if (err == DXL_LIB_OK) {
      tx_param[param_len++] = p_info->addr >> 0;
      tx_param[param_len++] = p_info->addr >> 8;
      tx_param[param_len++] = p_info->addr_length >> 0;
      tx_param[param_len++] = p_info->addr_length >> 8;
      err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
      if (err == DXL_LIB_OK) {
        for(uint8_t i = 0; i < p_info->xel_count; i++) {
          p_xel = &p_info->p_xels[i];
          err = add_param_to_dxl_packet(&info_tx_packet_, &p_xel->id, 1);
          if (err != DXL_LIB_OK)
            break;
        }
        if (err == DXL_LIB_OK) {
          err = end_make_dxl_packet(&info_tx_packet_);
          if (err == DXL_LIB_OK) {
            p_info->packet.gen_length = info_tx_packet_.generated_packet_length;
            p_info->packet.is_completed = true;
            p_info->is_info_changed = false;
          }
        }
      }
    }
  }

  if (err == DXL_LIB_OK) {
    p_port_->write(p_packet_buf, p_info->packet.gen_length);
    if (fastRxStatusPacket(p_info, nullptr, timeout_ms) != nullptr)
        recv_cnt = p_info->xel_count;
  } else {
    p_info->packet.is_completed = false;
  }

  return recv_cnt;
}

// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#sync-write
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#sync-write
bool
Master::syncWrite(InfoSyncWriteInst_t* p_info)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t i, tx_param[4], param_len = 0;
  uint8_t* p_packet_buf = nullptr;
  uint16_t packet_buf_cap;
  XELInfoSyncWrite_t* p_xel;

  // Parameter exception handling
  if(p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(protocol_ver_idx_ != 2 && protocol_ver_idx_ != 1){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  if(p_info->packet.p_buf != nullptr){
    p_packet_buf = p_info->packet.p_buf;
    packet_buf_cap = p_info->packet.buf_capacity;
  }else{
    p_packet_buf = p_packet_buf_;
    packet_buf_cap = packet_buf_capacity_;
  }

  if(p_info->packet.p_buf == nullptr && info_tx_packet_.inst_idx != DXL_INST_SYNC_WRITE){
    p_info->packet.is_completed = false;
  }

  if(p_info->packet.is_completed == false || p_info->is_info_changed == true){
    err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_,
      DXL_INST_SYNC_WRITE, 0, p_packet_buf, packet_buf_cap);
    if(err == DXL_LIB_OK){
      if(protocol_ver_idx_ == 2){
        tx_param[param_len++] = p_info->addr >> 0;
        tx_param[param_len++] = p_info->addr >> 8;
        tx_param[param_len++] = p_info->addr_length >> 0;
        tx_param[param_len++] = p_info->addr_length >> 8;
      }else if(protocol_ver_idx_ == 1){
        tx_param[param_len++] = p_info->addr & 0xFF;
        tx_param[param_len++] = p_info->addr_length & 0xFF;
      }
      err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
      if(err == DXL_LIB_OK){
        for(i=0; i<p_info->xel_count; i++){
          p_xel = &p_info->p_xels[i];
          err = add_param_to_dxl_packet(&info_tx_packet_, &p_xel->id, 1);
          if(err == DXL_LIB_OK){
            err = add_param_to_dxl_packet(&info_tx_packet_, p_xel->p_data, p_info->addr_length);
            if(err != DXL_LIB_OK){
              break;
            }
          }else{
            break;
          }
        }
        if(err == DXL_LIB_OK){
          err = end_make_dxl_packet(&info_tx_packet_);
          if(err == DXL_LIB_OK){
            p_info->packet.gen_length = info_tx_packet_.generated_packet_length;
            p_info->packet.is_completed = true;      
            p_info->is_info_changed = false;
          }
        }
      }
    }
  }

  if(err == DXL_LIB_OK){
    p_port_->write(p_packet_buf, p_info->packet.gen_length);
    ret = true;
  }else{
    p_info->packet.is_completed = false;
  }
  last_lib_err_ = err;

  return ret;
}


// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#bulk-read
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#bulk-read
uint8_t
Master::bulkRead(InfoBulkReadInst_t* p_info, uint32_t timeout_ms)
{
  uint8_t i, recv_cnt = 0;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t tx_param[5], param_len = 0;
  uint8_t* p_packet_buf = nullptr;
  uint16_t packet_buf_cap;
  XELInfoBulkRead_t* p_xel;

  // Parameter exception handling
  if(p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(protocol_ver_idx_ != 2 && protocol_ver_idx_ != 1){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return 0;
  }

  if(p_info->packet.p_buf != nullptr){
    p_packet_buf = p_info->packet.p_buf;
    packet_buf_cap = p_info->packet.buf_capacity;
  }else{
    p_packet_buf = p_packet_buf_;
    packet_buf_cap = packet_buf_capacity_;
  }

  if(p_info->packet.p_buf == nullptr && info_tx_packet_.inst_idx != DXL_INST_BULK_READ){
    p_info->packet.is_completed = false;
  }

  if(p_info->packet.is_completed == false || p_info->is_info_changed == true){
    err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_,
      DXL_INST_BULK_READ, 0, p_packet_buf, packet_buf_cap);
    if(protocol_ver_idx_ == 1 && err == DXL_LIB_OK){
      tx_param[0] = 0x00;
      err = add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)tx_param, 1);
    }      
    if(err == DXL_LIB_OK){
      for(i=0; i<p_info->xel_count; i++){
        p_xel = &p_info->p_xels[i];
        param_len = 0;
        if(protocol_ver_idx_ == 2){
          tx_param[param_len++] = p_xel->id;
          tx_param[param_len++] = p_xel->addr >> 0;
          tx_param[param_len++] = p_xel->addr >> 8;
          tx_param[param_len++] = p_xel->addr_length >> 0;
          tx_param[param_len++] = p_xel->addr_length >> 8;
        }else if(protocol_ver_idx_ == 1){
          tx_param[param_len++] = p_xel->id;
          tx_param[param_len++] = p_xel->addr & 0xFF;
          tx_param[param_len++] = p_xel->addr_length & 0xFF;
        }
        err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
        if(err != DXL_LIB_OK){
          break;
        }
      }
      if(err == DXL_LIB_OK){
        err = end_make_dxl_packet(&info_tx_packet_);
        if(err == DXL_LIB_OK){
          p_info->packet.gen_length = info_tx_packet_.generated_packet_length;
          p_info->packet.is_completed = true;      
          p_info->is_info_changed = false;
        }
      }
    }
  }

  if(err == DXL_LIB_OK){
    p_port_->write(p_packet_buf, p_info->packet.gen_length);

    for(i=0; i<p_info->xel_count; i++)
    {
      p_xel = &p_info->p_xels[i];
      if(rxStatusPacket(p_xel->p_recv_buf, p_xel->addr_length, timeout_ms) != nullptr){
        if(info_rx_packet_.id == p_xel->id){
          p_xel->error = info_rx_packet_.err_idx;
          recv_cnt++;
        }else{
          break;
        }
      }else{
        break;
      }
    }
  }else{
    p_info->packet.is_completed = false;
  }
  last_lib_err_ = err;

  return recv_cnt;
}


// (Protocol 1.0) Not Supported
// (Protocol 2.0) Refer to https://emanual.robotis.com/docs/en/dxl/protocol2/#fast-bulk-read-0x9a
uint8_t Master::fastBulkRead(InfoBulkReadInst_t* p_info, uint32_t timeout_ms)
{
  uint8_t i, recv_cnt = 0;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t tx_param[5], param_len = 0;
  uint8_t* p_packet_buf = nullptr;
  uint16_t packet_buf_cap;
  XELInfoBulkRead_t* p_xel;

  // Parameter exception handling
  if (p_port_ == nullptr)
    err = DXL_LIB_ERROR_NULLPTR;
  else if (p_port_->getOpenState() != true)
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  else if (protocol_ver_idx_ != 2)
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  if (err != DXL_LIB_OK) {
    last_lib_err_ = err;
    return 0;
  }

  if (p_info->packet.p_buf != nullptr) {
    p_packet_buf = p_info->packet.p_buf;
    packet_buf_cap = p_info->packet.buf_capacity;
  } else {
    p_packet_buf = p_packet_buf_;
    packet_buf_cap = packet_buf_capacity_;
  }

  if ((p_info->packet.p_buf == nullptr) && (info_tx_packet_.inst_idx != DXL_INST_FAST_BULK_READ))
    p_info->packet.is_completed = false;

  if ((p_info->packet.is_completed == false) || (p_info->is_info_changed == true)) {
    err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_,
                                DXL_INST_FAST_BULK_READ, 0, p_packet_buf, packet_buf_cap);
    if (err == DXL_LIB_OK) {
      for (i = 0; i < p_info->xel_count; i++) {
        p_xel = &p_info->p_xels[i];
        param_len = 0;
        tx_param[param_len++] = p_xel->id;
        tx_param[param_len++] = p_xel->addr >> 0;
        tx_param[param_len++] = p_xel->addr >> 8;
        tx_param[param_len++] = p_xel->addr_length >> 0;
        tx_param[param_len++] = p_xel->addr_length >> 8;
        err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
        if (err != DXL_LIB_OK)
          break;
      }
      if (err == DXL_LIB_OK) {
        err = end_make_dxl_packet(&info_tx_packet_);
        if (err == DXL_LIB_OK) {
          p_info->packet.gen_length = info_tx_packet_.generated_packet_length;
          p_info->packet.is_completed = true;      
          p_info->is_info_changed = false;
        }
      }
    }
  }

  if (err == DXL_LIB_OK) {
    p_port_->write(p_packet_buf, p_info->packet.gen_length);
    if (fastRxStatusPacket(nullptr, p_info, timeout_ms) != nullptr)
        recv_cnt = p_info->xel_count;
  } else {
    p_info->packet.is_completed = false;
  }

  return recv_cnt;
}

// (Protocol 1.0) Not Supported
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#bulk-write
bool
Master::bulkWrite(InfoBulkWriteInst_t* p_info)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t i, tx_param[5], param_len = 0;
  uint8_t* p_packet_buf = nullptr;
  uint16_t packet_buf_cap;
  XELInfoBulkWrite_t* p_xel;

  // Parameter exception handling
  if(p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(protocol_ver_idx_ != 2){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return 0;
  }

  if(p_info->packet.p_buf != nullptr){
    p_packet_buf = p_info->packet.p_buf;
    packet_buf_cap = p_info->packet.buf_capacity;
  }else{
    p_packet_buf = p_packet_buf_;
    packet_buf_cap = packet_buf_capacity_;
  }

  if(p_info->packet.p_buf == nullptr && info_tx_packet_.inst_idx != DXL_INST_BULK_WRITE){
    p_info->packet.is_completed = false;
  }

  if(p_info->packet.is_completed == false || p_info->is_info_changed == true){
    err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_,
      DXL_INST_BULK_WRITE, 0, p_packet_buf, packet_buf_cap);
    if(err == DXL_LIB_OK){
      for(i=0; i<p_info->xel_count; i++){
        p_xel = &p_info->p_xels[i];
        param_len = 0;
        tx_param[param_len++] = p_xel->id;
        tx_param[param_len++] = p_xel->addr >> 0;
        tx_param[param_len++] = p_xel->addr >> 8;
        tx_param[param_len++] = p_xel->addr_length >> 0;
        tx_param[param_len++] = p_xel->addr_length >> 8;
        err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
        if(err == DXL_LIB_OK){
          err = add_param_to_dxl_packet(&info_tx_packet_, p_xel->p_data, p_xel->addr_length);
        }
        if(err != DXL_LIB_OK){
          break;
        }
      }
      if(err == DXL_LIB_OK){
        err = end_make_dxl_packet(&info_tx_packet_);
        if(err == DXL_LIB_OK){
          p_info->packet.gen_length = info_tx_packet_.generated_packet_length;
          p_info->packet.is_completed = true;
          p_info->is_info_changed = false;
        }
      }
    }
  }

  if(err == DXL_LIB_OK){
    p_port_->write(p_packet_buf, p_info->packet.gen_length);
    ret = true;
  }else{
    p_info->packet.is_completed = false;
  }
  last_lib_err_ = err;

  return ret;
}



// (Protocol 1.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol1/#error
// (Protocol 2.0) Refer to http://emanual.robotis.com/docs/en/dxl/protocol2/#error
uint8_t 
Master::getLastStatusPacketError() const
{
  return info_rx_packet_.err_idx;
}


DXLLibErrorCode_t 
Master::getLastLibErrCode() const
{
  return last_lib_err_;
}

void
Master::setLastLibErrCode(DXLLibErrorCode_t err_code)
{
  last_lib_err_ = err_code;
}


bool 
Master::txInstPacket(uint8_t id, uint8_t inst_idx, uint8_t *p_param, uint16_t param_len)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;

  // Parameter exception handling
  if(p_port_ == nullptr
  || (param_len > 0 && p_param == nullptr)){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(inst_idx == DXL_INST_STATUS){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  // Send Instruction Packet
  err = begin_make_dxl_packet(&info_tx_packet_, id, protocol_ver_idx_,
    inst_idx, 0, p_packet_buf_, packet_buf_capacity_);
  if(err == DXL_LIB_OK){
    err = add_param_to_dxl_packet(&info_tx_packet_, p_param, param_len);
    if(err == DXL_LIB_OK){
      err = end_make_dxl_packet(&info_tx_packet_);
    }
  }
  if(err == DXL_LIB_OK){
    if(p_port_->write(info_tx_packet_.p_packet_buf, info_tx_packet_.generated_packet_length)
        == info_tx_packet_.generated_packet_length)
    {
      ret = true;
    }else{
      err = DXL_LIB_ERROR_PORT_WRITE;
    }
  }

  last_lib_err_ = err;

  return ret;
}


const InfoToParseDXLPacket_t* 
Master::rxStatusPacket(uint8_t* p_param_buf, uint16_t param_buf_cap, uint32_t timeout_ms)
{
  InfoToParseDXLPacket_t *p_ret = nullptr;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint32_t pre_time_ms;

  // Parameter exception handling
  if(p_port_ == nullptr
  || (param_buf_cap > 0 && p_param_buf == nullptr)){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return nullptr;
  }

  // Receive Status Packet
  err = begin_parse_dxl_packet(&info_rx_packet_, protocol_ver_idx_, p_param_buf, param_buf_cap);
  if(err == DXL_LIB_OK){
    pre_time_ms = millis();
    while(1) 
    {
      if(p_port_->available() > 0){
        err = parse_dxl_packet(&info_rx_packet_, p_port_->read());
        if(err == DXL_LIB_OK){
          if((protocol_ver_idx_ == 2 && info_rx_packet_.inst_idx == DXL_INST_STATUS)
          || protocol_ver_idx_ == 1){
            p_ret = &info_rx_packet_;
            break;
          }
        }else if(err != DXL_LIB_PROCEEDING){
          break;
        }
      }

      if (millis()-pre_time_ms >= timeout_ms) {
        err = DXL_LIB_ERROR_TIMEOUT;
        break;
      }
    }
  }

  last_lib_err_ = err;

  return p_ret;
}

const InfoToParseDXLPacket_t* Master::fastRxStatusPacket(InfoSyncReadInst_t* sync_read, InfoBulkReadInst_t* bulk_read, uint32_t timeout_ms)
{
  InfoToParseDXLPacket_t *p_ret = nullptr;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint32_t pre_time_ms;

  // Parameter exception handling
  if (p_port_ == nullptr)
    err = DXL_LIB_ERROR_NULLPTR;
  else if (p_port_->getOpenState() != true)
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  if (nullptr != sync_read) {
    for (uint8_t index = 0; index < sync_read->xel_count; index++) {
      if ((sync_read->addr_length > 0) && (sync_read->p_xels[index].p_recv_buf == nullptr)) {
        err = DXL_LIB_ERROR_NULLPTR;
        break;
      }
    }
  } else if (nullptr != bulk_read) {
    for (uint8_t index = 0; index < bulk_read->xel_count; index++) {
      if ((bulk_read->p_xels[index].addr_length > 0) && (bulk_read->p_xels[index].p_recv_buf == nullptr)) {
        err = DXL_LIB_ERROR_NULLPTR;
        break;
      }
    }
  }
  if (err != DXL_LIB_OK) {
    last_lib_err_ = err;
    return nullptr;
  }

  // Receive Status Packet
  err = fast_begin_parse_dxl_packet(&info_rx_packet_, protocol_ver_idx_);
  info_rx_packet_.param_length = 0;
  if (nullptr != sync_read) {
    info_rx_packet_.param_length = (sync_read->addr_length + 4) * sync_read->xel_count - 3; // 4 = Error(1) + ID(1) + CRC(2), 3 = Error(1) + CRC(2) 
  } else if (nullptr != bulk_read) {
    for (uint8_t i = 0; i < bulk_read->xel_count; i++) {
      info_rx_packet_.param_length += (bulk_read->p_xels[i].addr_length + 4); // 4 = Error(1) + ID(1) + CRC(2)
    }
    info_rx_packet_.param_length -= 3; // 3 = Error(1) + CRC(2) 
  }
  if (err == DXL_LIB_OK) {
    pre_time_ms = millis();
    while (true) {
      if (p_port_->available() > 0) {
        err = fast_parse_dxl_packet(&info_rx_packet_, p_port_->read(), sync_read, bulk_read);
        if (err == DXL_LIB_OK) {
          if (info_rx_packet_.inst_idx == DXL_INST_STATUS) {
            p_ret = &info_rx_packet_;
            break;
          }
        } else if (err != DXL_LIB_PROCEEDING) {
          break;
        }
      }
      if (millis()-pre_time_ms >= timeout_ms) {
        err = DXL_LIB_ERROR_TIMEOUT;
        break;
      }
    }
  }

  last_lib_err_ = err;
  return p_ret;
}



// >> Legacy (Deprecated since v0.4.0)
bool Master::syncRead(const ParamForSyncReadInst_t &param_info, RecvInfoFromStatusInst_t &recv_info, uint32_t timeout_ms)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t i, tx_param[4], param_len = 0;

  // Parameter exception handling
  if(p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(protocol_ver_idx_ != 2){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_,
      DXL_INST_SYNC_READ, 0, p_packet_buf_, packet_buf_capacity_);
  if(err == DXL_LIB_OK){
    tx_param[param_len++] = param_info.addr >> 0;
    tx_param[param_len++] = param_info.addr >> 8;
    tx_param[param_len++] = param_info.length >> 0;
    tx_param[param_len++] = param_info.length >> 8;
    err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
    if(err == DXL_LIB_OK){
      for(i=0; i<param_info.id_count; i++){
        err = add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)&param_info.xel[i].id, 1);
        if(err != DXL_LIB_OK){
          break;
        }
      }
      if(err == DXL_LIB_OK){
        err = end_make_dxl_packet(&info_tx_packet_);
      }
    }
  }

  if(err == DXL_LIB_OK){
    p_port_->write(p_packet_buf_, info_tx_packet_.generated_packet_length);

    for(i=0; i<param_info.id_count; i++)
    {
      if(rxStatusPacket(recv_info.xel[i].data, DXL_MAX_NODE_BUFFER_SIZE, timeout_ms) != nullptr){
        if(info_rx_packet_.id == param_info.xel[i].id){
          recv_info.xel[i].id     = info_rx_packet_.id;
          recv_info.xel[i].error  = info_rx_packet_.err_idx;
          recv_info.xel[i].length = info_rx_packet_.recv_param_len;
        }else{
          break;
        }
      }else{
        break;
      }
    }
    recv_info.id_count = i;
    if(recv_info.id_count > 0){
      ret = true;
    }
  }
  last_lib_err_ = err;
 
  return ret;
}


bool Master::syncWrite(const ParamForSyncWriteInst_t &param_info)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t i, tx_param[4], param_len = 0;
  
  // Parameter exception handling
  if(p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(protocol_ver_idx_ != 2 && protocol_ver_idx_ != 1){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_,
    DXL_INST_SYNC_WRITE, 0, p_packet_buf_, packet_buf_capacity_);
  if(err == DXL_LIB_OK){
    if(protocol_ver_idx_ == 2){
      tx_param[param_len++] = param_info.addr >> 0;
      tx_param[param_len++] = param_info.addr >> 8;
      tx_param[param_len++] = param_info.length >> 0;
      tx_param[param_len++] = param_info.length >> 8;
    }else if(protocol_ver_idx_ == 1){
      tx_param[param_len++] = param_info.addr & 0xFF;
      tx_param[param_len++] = param_info.length & 0xFF;
    }
    err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
    if(err == DXL_LIB_OK){
      for(i=0; i<param_info.id_count; i++){
        err = add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)&param_info.xel[i].id, 1);
        if(err == DXL_LIB_OK){
          err = add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)param_info.xel[i].data, param_info.length);
          if(err != DXL_LIB_OK){
            break;
          }
        }else{
          break;
        }
      }
      if(err == DXL_LIB_OK){
        err = end_make_dxl_packet(&info_tx_packet_);
      }
    }
  }

  if(err == DXL_LIB_OK){
    p_port_->write(p_packet_buf_, info_tx_packet_.generated_packet_length);
    ret = true;
  }
  last_lib_err_ = err;

  return ret;
}


bool Master::bulkRead(const ParamForBulkReadInst_t &param_info, RecvInfoFromStatusInst_t &recv_info, uint32_t timeout_ms)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t i, tx_param[5], param_len = 0;
  const XelInfoForBulkReadParam_t* p_xel;

  // Parameter exception handling
  if(p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(protocol_ver_idx_ != 2 && protocol_ver_idx_ != 1){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return 0;
  }

  err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_,
    DXL_INST_BULK_READ, 0, p_packet_buf_, packet_buf_capacity_);
  if(protocol_ver_idx_ == 1 && err == DXL_LIB_OK){
    tx_param[0] = 0x00;
    err = add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)tx_param, 1);
  }      
  if(err == DXL_LIB_OK){
    for(i=0; i<param_info.id_count; i++){
      p_xel = &param_info.xel[i];
      param_len = 0;
      if(protocol_ver_idx_ == 2){
        tx_param[param_len++] = p_xel->id;
        tx_param[param_len++] = p_xel->addr >> 0;
        tx_param[param_len++] = p_xel->addr >> 8;
        tx_param[param_len++] = p_xel->length >> 0;
        tx_param[param_len++] = p_xel->length >> 8;
      }else if(protocol_ver_idx_ == 1){
        tx_param[param_len++] = p_xel->length & 0xFF;
        tx_param[param_len++] = p_xel->id;
        tx_param[param_len++] = p_xel->addr & 0xFF;
      }
      err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
      if(err != DXL_LIB_OK){
        break;
      }
    }
    if(err == DXL_LIB_OK){
      err = end_make_dxl_packet(&info_tx_packet_);
    }
  }

  if(err == DXL_LIB_OK){
    p_port_->write(p_packet_buf_, info_tx_packet_.generated_packet_length);

    for(i=0; i<param_info.id_count; i++)
    {
      p_xel = &param_info.xel[i];
      if(rxStatusPacket(recv_info.xel[i].data, p_xel->length, timeout_ms) != nullptr){
        if(info_rx_packet_.id == param_info.xel[i].id){
          recv_info.xel[recv_info.id_count].id     = info_rx_packet_.id;
          recv_info.xel[recv_info.id_count].error  = info_rx_packet_.err_idx;
          recv_info.xel[recv_info.id_count].length = info_rx_packet_.recv_param_len;
          recv_info.id_count++;
        }else{
          break;
        }
      }else{
        break;
      }
    }
    if(i == param_info.id_count){
      ret = true;
    }
  }
  last_lib_err_ = err;


  if(err == DXL_LIB_OK){
    p_port_->write(p_packet_buf_, info_tx_packet_.generated_packet_length);

    for(i=0; i<param_info.id_count; i++)
    {
      if(rxStatusPacket(recv_info.xel[i].data, DXL_MAX_NODE_BUFFER_SIZE, timeout_ms) != nullptr){
        if(info_rx_packet_.id == param_info.xel[i].id){
          recv_info.xel[i].id     = info_rx_packet_.id;
          recv_info.xel[i].error  = info_rx_packet_.err_idx;
          recv_info.xel[i].length = info_rx_packet_.recv_param_len;
        }else{
          break;
        }
      }else{
        break;
      }
    }
    recv_info.id_count = i;
    if(recv_info.id_count > 0){
      ret = true;
    }
  }

  return ret;
}


bool Master::bulkWrite(const ParamForBulkWriteInst_t &param_info)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  uint8_t i, tx_param[5], param_len = 0;
  const XelInfoForBulkWriteParam_t* p_xel;

  // Parameter exception handling
  if(p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }else if(protocol_ver_idx_ != 2){
    err = DXL_LIB_ERROR_NOT_SUPPORTED;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return 0;
  }

  err = begin_make_dxl_packet(&info_tx_packet_, DXL_BROADCAST_ID, protocol_ver_idx_,
    DXL_INST_BULK_WRITE, 0, p_packet_buf_, packet_buf_capacity_);
  if(err == DXL_LIB_OK){
    for(i=0; i<param_info.id_count; i++){
      p_xel = &param_info.xel[i];
      param_len = 0;
      tx_param[param_len++] = p_xel->id;
      tx_param[param_len++] = p_xel->addr >> 0;
      tx_param[param_len++] = p_xel->addr >> 8;
      tx_param[param_len++] = p_xel->length >> 0;
      tx_param[param_len++] = p_xel->length >> 8;
      err = add_param_to_dxl_packet(&info_tx_packet_, tx_param, param_len);
      if(err == DXL_LIB_OK){
        err = add_param_to_dxl_packet(&info_tx_packet_, (uint8_t*)p_xel->data, p_xel->length);
      }
      if(err != DXL_LIB_OK){
        break;
      }
    }
    if(err == DXL_LIB_OK){
      err = end_make_dxl_packet(&info_tx_packet_);
    }
  }

  if(err == DXL_LIB_OK){
    p_port_->write(p_packet_buf_, info_tx_packet_.generated_packet_length);
    ret = true;
  }
  last_lib_err_ = err;

  return ret;
}
// << Legacy (Deprecated since v0.4.0)