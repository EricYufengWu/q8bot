#include "slave.h"

using namespace DYNAMIXEL;


enum DefaultControlTableItemAddr{
  ADDR_ITEM_MODEL_NUMBER    = 0,
  ADDR_ITEM_FIRMWARE_VER    = 6,
  ADDR_ITEM_ID              = 7,
  ADDR_ITEM_PROTOCOL_VER    = 9
};

static bool isAddrInRange(uint16_t addr, uint16_t length, uint16_t range_addr, uint16_t range_length);
static bool isAddrInOtherItem(uint16_t start_addr, uint16_t length, uint16_t other_start_addr, uint16_t other_length);


Slave::Slave(DXLPortHandler &port, const uint16_t model_num, float protocol_ver)
: model_num_(model_num), protocol_ver_idx_(2), firmware_ver_(1), id_(1),
  is_buf_malloced_(false), packet_buf_capacity_(0),
  last_lib_err_(DXL_LIB_OK),
  registered_item_cnt_(0)
{
  setPort(port);
  setPortProtocolVersion(protocol_ver);
  addDefaultControlItem();

  p_packet_buf_ = new uint8_t[DEFAULT_DXL_BUF_LENGTH + DXL_BYTE_STUFF_SAFE_CNT];
  if(p_packet_buf_ != nullptr){
    packet_buf_capacity_ = DEFAULT_DXL_BUF_LENGTH;
    is_buf_malloced_ = true;
  }
  info_tx_packet_.is_init = false;  
  info_rx_packet_.is_init = false;
}

Slave::Slave(const uint16_t model_num, float protocol_ver)
: model_num_(model_num), protocol_ver_idx_(2), firmware_ver_(1), id_(1),
  is_buf_malloced_(false), packet_buf_capacity_(0),
  last_lib_err_(DXL_LIB_OK),
  registered_item_cnt_(0)
{
  setPortProtocolVersion(protocol_ver);
  addDefaultControlItem();

  p_packet_buf_ = new uint8_t[DEFAULT_DXL_BUF_LENGTH + DXL_BYTE_STUFF_SAFE_CNT];
  if(p_packet_buf_ != nullptr){
    packet_buf_capacity_ = DEFAULT_DXL_BUF_LENGTH;
    is_buf_malloced_ = true;
  }  
  info_tx_packet_.is_init = false;
  info_rx_packet_.is_init = false;
}


bool 
Slave::setPacketBuffer(uint8_t* p_buf, uint16_t buf_capacity)
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
Slave::getPacketBuffer() const
{
  return p_packet_buf_;
}

uint16_t 
Slave::getPacketBufferCapacity() const
{
  return packet_buf_capacity_;
}

bool 
Slave::setPortProtocolVersion(float version)
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
Slave::setPortProtocolVersionUsingIndex(uint8_t version_idx)
{
  if(version_idx != 2 && version_idx != 1){
    return false;
  }
  protocol_ver_idx_ = version_idx;

  return true;
}

float 
Slave::getPortProtocolVersion() const
{
  return (float)protocol_ver_idx_;
}

void
Slave::begin(void)
{
  p_port_->begin(); 
}

uint8_t 
Slave::getPortProtocolVersionIndex() const
{
  return protocol_ver_idx_;
}

uint16_t
Slave::getModelNumber() const
{
  return model_num_;
}

bool 
Slave::setID(uint8_t id)
{
  DXLLibErrorCode_t err = DXL_LIB_OK;

  if(protocol_ver_idx_ == 2 && id >= 0xFD){ //http://emanual.robotis.com/docs/en/dxl/protocol2/#packet-id
    err = DXL_LIB_ERROR_INVAILD_ID;
  }else if(protocol_ver_idx_ == 1 && id >= DXL_BROADCAST_ID){ //http://emanual.robotis.com/docs/en/dxl/protocol1/#packet-id
    err = DXL_LIB_ERROR_INVAILD_ID;
  }
  last_lib_err_ = err; 
  if(err != DXL_LIB_OK){
    return false;
  }

  id_ = id;

  return true;
}

uint8_t 
Slave::getID() const
{
  return id_;
}

void 
Slave::setFirmwareVersion(uint8_t version)
{
  firmware_ver_ = version;
}

uint8_t 
Slave::getFirmwareVersion() const
{
  return firmware_ver_;
}

bool 
Slave::processPacket()
{
  bool ret = true;

  if(rxInstPacket(p_packet_buf_, packet_buf_capacity_) != nullptr){
    ret = processInst(info_rx_packet_.inst_idx);
  }

  return ret;
}


uint8_t Slave::getNumCanBeRegistered() const
{
  return CONTROL_ITEM_MAX-registered_item_cnt_;
}

bool Slave::isEnoughSpaceInControlTable(uint16_t start_addr, uint16_t length)
{
  uint16_t available_start_addr = control_table_[registered_item_cnt_].start_addr + control_table_[registered_item_cnt_].length;

  if(start_addr > CONTROL_ITEM_ADDR_LIMIT){
    last_lib_err_ = DXL_LIB_ERROR_INVAILD_ADDR;
    return false;
  }

  if(length == 0 || length > CONTROL_ITEM_ADDR_LIMIT - available_start_addr){
    last_lib_err_ = DXL_LIB_ERROR_ADDR_LENGTH;
    return false;
  }

  return true;
}


uint8_t Slave::addControlItem(uint16_t start_addr, uint8_t* p_data, uint16_t length)
{
  if(registered_item_cnt_ >= CONTROL_ITEM_MAX){
    last_lib_err_ = DXL_LIB_ERROR_BUFFER_OVERFLOW;
    return last_lib_err_;
  }

  if(p_data == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return last_lib_err_;
  }

  if(isEnoughSpaceInControlTable(start_addr, length) == false){
    return last_lib_err_;
  }

  for(uint16_t i=0; i < registered_item_cnt_; i++){
    if(isAddrInOtherItem(start_addr, length, control_table_[i].start_addr, control_table_[i].length)){
      last_lib_err_ = DXL_LIB_ERROR_INVAILD_ADDR;
      return last_lib_err_;
    }
  }

  control_table_[registered_item_cnt_].start_addr = start_addr;
  control_table_[registered_item_cnt_].length = length;
  control_table_[registered_item_cnt_].p_data = p_data;

  registered_item_cnt_++;

  last_lib_err_ = DXL_LIB_OK;
  
  return last_lib_err_;
}

uint8_t Slave::addControlItem(uint16_t start_addr, bool &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, uint8_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, uint16_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, uint32_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, uint64_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, int8_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, int16_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, int32_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, int64_t &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, float &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}

uint8_t Slave::addControlItem(uint16_t start_addr, double &data)
{
  return addControlItem(start_addr, (uint8_t*)&data, (uint16_t)sizeof(data));
}




void Slave::setWriteCallbackFunc(userCallbackFunc callback_func, void* callback_arg)
{
  user_write_callback_ = callback_func;
  user_write_callbakc_arg_ = callback_arg;
}

void Slave::setReadCallbackFunc(userCallbackFunc callback_func, void* callback_arg)
{
  user_read_callback_ = callback_func;
  user_read_callbakc_arg_ = callback_arg;
}

bool 
Slave::setPort(DXLPortHandler *p_port)
{
  if(p_port == nullptr){
    last_lib_err_ = DXL_LIB_ERROR_NULLPTR;
    return false;
  }

  p_port_ = p_port;

  return true;
}

bool 
Slave::setPort(DXLPortHandler &port)
{
  p_port_ = &port;

  return true;
}

DXLPortHandler* 
Slave::getPort() const
{
  return p_port_;
}

DXLLibErrorCode_t
Slave::getLastLibErrCode() const
{
  return last_lib_err_;
}

void
Slave::setLastLibErrCode(DXLLibErrorCode_t err_code)
{
  last_lib_err_ = err_code;
}

uint8_t 
Slave::getLastStatusPacketError() const
{
  return info_tx_packet_.err_idx;
}


bool 
Slave::processInstPing()
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  InfoToParseDXLPacket_t *p_rx_info;  
  uint8_t tx_param[3];
  uint16_t tx_param_len = 0;
  
  p_rx_info = &info_rx_packet_;

  if(p_rx_info->id != DXL_BROADCAST_ID){
    if(p_rx_info->protocol_ver == 2){
      tx_param_len = 3;
      if(tx_param_len+11 <= packet_buf_capacity_){
        tx_param[0] = (uint8_t)(model_num_ >> 0);
        tx_param[1] = (uint8_t)(model_num_ >> 8);
        tx_param[2] = (uint8_t)firmware_ver_;        
      }else{
        err = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
      }
    }else if(p_rx_info->protocol_ver == 1){
      //
    }else{
      err = DXL_LIB_ERROR_WRONG_PACKET;
    }
    if(err == DXL_LIB_OK){
      ret = txStatusPacket(id_, 0, tx_param, tx_param_len);
    }
  }else{
    err = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
  }

  last_lib_err_ = err;

  return ret;
}


bool 
Slave::processInstRead()
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  InfoToParseDXLPacket_t *p_rx_info;
  uint8_t packet_err = 0;
  uint8_t *p_rx_param, *p_tx_param;
  uint16_t addr, addr_length = 0;

  // Parameter exception handling
  if(p_port_ == nullptr){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  p_rx_info = &info_rx_packet_;
  p_rx_param = p_rx_info->p_param_buf;

  if(p_rx_info->id != DXL_BROADCAST_ID){
    if(p_rx_info->protocol_ver == 2){
      if(p_rx_info->recv_param_len == 4){ //4 = Address(2)+AddressLength(2)
        addr = ((uint16_t)p_rx_param[1]<<8) | (uint16_t)p_rx_param[0];
        addr_length = ((uint16_t)p_rx_param[3]<<8) | (uint16_t)p_rx_param[2];
        if(addr_length+11 > packet_buf_capacity_){
          err = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
        }
        p_tx_param = &p_packet_buf_[9 + DXL_BYTE_STUFF_SAFE_CNT];
      }else{
        err = DXL_LIB_ERROR_WRONG_PACKET;
      }
    }else if(p_rx_info->protocol_ver == 1){
      if(p_rx_info->recv_param_len == 2){ //2 = Address(1)+AddressLength(1)
        addr = p_rx_param[0];
        addr_length = p_rx_param[1];
        if(addr_length+6 > packet_buf_capacity_){
          err = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
        }
        p_tx_param = &p_packet_buf_[5 + DXL_BYTE_STUFF_SAFE_CNT];
      }else{
        err = DXL_LIB_ERROR_WRONG_PACKET;
      }
    }else{
      err = DXL_LIB_ERROR_WRONG_PACKET;
    }

    if(err == DXL_LIB_OK){
      uint8_t i, j;
      uint16_t item_start_addr, item_addr_length;
      ControlItem_t *p_item;
      memset(p_packet_buf_, 0, packet_buf_capacity_);
      for(i=0; i < registered_item_cnt_; i++){
        p_item = &control_table_[i];
        item_start_addr = p_item->start_addr;
        item_addr_length = p_item->length;
        if(item_addr_length != 0
        && p_item->p_data != nullptr
        && isAddrInRange(item_start_addr, item_addr_length, addr, addr_length) == true){
          if(user_read_callback_ != nullptr){
            user_read_callback_(item_start_addr, packet_err, user_read_callbakc_arg_);
            if(packet_err != 0){      
              break;
            }
          }
          for(j=0; j<item_addr_length; j++){
            p_tx_param[item_start_addr-addr+j] = p_item->p_data[j];
          }
        }
      }
      ret = txStatusPacket(id_, packet_err, p_tx_param, addr_length);
    }
  }else{
    err = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
  }

  last_lib_err_ = err;

  return ret;
}


bool
Slave::processInstWrite()
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;
  InfoToParseDXLPacket_t *p_rx_info;
  uint8_t *p_rx_param, *p_data;
  uint8_t packet_err = 0;
  uint16_t addr, data_length = 0;

  p_rx_info = &info_rx_packet_;
  p_rx_param = p_rx_info->p_param_buf;

  if(p_rx_info->id == DXL_BROADCAST_ID){
    last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORT_BROADCAST;
    return false;
  }

  // extract start address and length from the instruction packet
  switch (p_rx_info->protocol_ver)
  {
  case 2:
    if(p_rx_info->recv_param_len <= 2) { //2 = Address(2)+Data(n)
      err = DXL_LIB_ERROR_WRONG_PACKET;
    } 
    else {
      addr = ((uint16_t)p_rx_param[1]<<8) | (uint16_t)p_rx_param[0];
      p_data = &p_rx_param[2];
      data_length = p_rx_info->recv_param_len-2;
      if(data_length+11 > packet_buf_capacity_){
        err = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
      }
    }
    break;
  
  case 1:  
    if(p_rx_info->recv_param_len <= 1){ //1 = Address(1)+Data(n)
      err = DXL_LIB_ERROR_WRONG_PACKET;
    }
    else {
      addr = p_rx_param[0];
      p_data = &p_rx_param[1];
      data_length = p_rx_info->recv_param_len-1;
      if(data_length+6 > packet_buf_capacity_){
        err = DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
      }
    }
    break;

  default:
    err = DXL_LIB_ERROR_WRONG_PACKET;
    break;
  }

  if(err == DXL_LIB_OK){
    uint8_t i, j;
    uint8_t backup_data[32];      // we max support registers of 32 bytes
    uint16_t item_start_addr, item_addr_length;
    ControlItem_t *p_item;

    for(i=0; i < registered_item_cnt_; i++){

      p_item = &control_table_[i];
      item_start_addr = p_item->start_addr;
      item_addr_length = p_item->length;

      if(item_addr_length != 0 && p_item->p_data != nullptr
      && isAddrInRange(item_start_addr, item_addr_length, addr, data_length) == true){

        // backup data, copy new data
        for(j=0; j<item_addr_length; j++){
          // backup supported only for registers smaller than 32 bytes
          if (item_addr_length <= 32) {
            backup_data[j] = p_item->p_data[j];
          }
          p_item->p_data[j] = p_data[item_start_addr - addr + j];
        }
        // Check data for ID, Protocol Version (Act as a system callback)
        switch (item_start_addr) {

          case ADDR_ITEM_ID:      // validate ID
            if(protocol_ver_idx_ == 2 && id_ >= 0xFD){
              packet_err = DXL2_0_ERR_DATA_RANGE;
            }
            if(protocol_ver_idx_ == 1 && id_ >= 0xFE){
              packet_err |= 1<<DXL1_0_ERR_RANGE_BIT;
            }   
            break;
          
          case ADDR_ITEM_PROTOCOL_VER: // validate Protocol Version 
            if(protocol_ver_idx_ != 1 && protocol_ver_idx_ != 2){
              if(backup_data[0] == 2){
                packet_err = DXL2_0_ERR_DATA_RANGE;
              }
              else if(backup_data[0] == 1){
                packet_err |= 1<<DXL1_0_ERR_RANGE_BIT;
              }
            }
            break;

          case ADDR_ITEM_MODEL_NUMBER:  // model number if Read Only
          case ADDR_ITEM_FIRMWARE_VER:  // firmware is Read Only
            if(backup_data[0] == 2){
              packet_err = DXL2_0_ERR_DATA_RANGE;
            }
            else if(backup_data[0] == 1){
              packet_err |= 1<<DXL1_0_ERR_RANGE_BIT;
            }
            break;
          
        }

        // Run user callback for Write instruction
        //
        // NOTE: Slave does not implement EEPROM persistence of data.
        // If your device needs to persist data to ROM you must implement
        // `user_write_callback_` and there save the registers to ROM
        // including the ones implemented in Slave (id, protocol, 
        // model number, firmware version)
        if (packet_err == 0 && user_write_callback_ != nullptr){
          user_write_callback_(item_start_addr, packet_err, user_write_callbakc_arg_);
        }

        if(packet_err != 0){
          // If an error occurs restore the previous data.
          // backup supported only for registers smaller than 32 bytes
          if (item_addr_length <= 32) {
            for(j=0; j<item_addr_length; j++){
              p_item->p_data[j] = backup_data[j];
            }
          }
        }

      } // if

    } // for

    ret = txStatusPacket(id_, packet_err, nullptr, 0);
  }

  last_lib_err_ = err;

  return ret;
}

bool 
Slave::addDefaultControlItem()
{
  if(addControlItem(ADDR_ITEM_MODEL_NUMBER, (uint16_t&)model_num_) != DXL_LIB_OK
  || addControlItem(ADDR_ITEM_FIRMWARE_VER, firmware_ver_) != DXL_LIB_OK
  || addControlItem(ADDR_ITEM_ID, id_) != DXL_LIB_OK
  || addControlItem(ADDR_ITEM_PROTOCOL_VER, protocol_ver_idx_) != DXL_LIB_OK){
    return false;
  }   

  return true;
}

bool 
Slave::processInst(uint8_t inst_idx)
{
  bool ret = false;

  switch(inst_idx)
  {
    case DXL_INST_PING:
      ret = processInstPing();
      break;

    case DXL_INST_READ:
      ret = processInstRead();
      break;

    case DXL_INST_WRITE:
      ret = processInstWrite();
      break;

    default:
      last_lib_err_ = DXL_LIB_ERROR_NOT_SUPPORTED;
      break;  
  }

  return ret;
}


bool 
Slave::txStatusPacket(uint8_t id, uint8_t err_code, uint8_t *p_param, uint16_t param_len)
{
  bool ret = false;
  DXLLibErrorCode_t err = DXL_LIB_OK;

  // Parameter exception handling
  if(p_port_ == nullptr
  || (param_len > 0 && p_param == nullptr)){
    err = DXL_LIB_ERROR_NULLPTR;
  }else if(p_port_->getOpenState() != true){
    err = DXL_LIB_ERROR_PORT_NOT_OPEN;
  }
  if(err != DXL_LIB_OK){
    last_lib_err_ = err;
    return false;
  }

  // Send Status Packet
  begin_make_dxl_packet(&info_tx_packet_, id, protocol_ver_idx_,
    DXL_INST_STATUS, err_code, p_packet_buf_, packet_buf_capacity_);   
  add_param_to_dxl_packet(&info_tx_packet_, p_param, param_len);
  err = end_make_dxl_packet(&info_tx_packet_);
  if(err == DXL_LIB_OK){
    p_port_->write(info_tx_packet_.p_packet_buf, info_tx_packet_.generated_packet_length);
    ret = true;
  }

  last_lib_err_ = err;

  return ret;
}

const InfoToParseDXLPacket_t* 
Slave::rxInstPacket(uint8_t* p_param_buf, uint16_t param_buf_cap)
{
  InfoToParseDXLPacket_t *p_ret = nullptr;
  DXLLibErrorCode_t err = DXL_LIB_OK;

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

  // Receive Instruction Packet
  begin_parse_dxl_packet(&info_rx_packet_, protocol_ver_idx_, p_param_buf, param_buf_cap);
  while(p_port_->available() > 0) 
  {
    err = parse_dxl_packet(&info_rx_packet_, p_port_->read());
    if(err == DXL_LIB_OK){
      if((protocol_ver_idx_ == 2 && info_rx_packet_.inst_idx != DXL_INST_STATUS)
      || protocol_ver_idx_ == 1){
        if(info_rx_packet_.id == id_ || info_rx_packet_.id == DXL_BROADCAST_ID){
          p_ret = &info_rx_packet_;
        }
        break;
      }
    }else if(err != DXL_LIB_PROCEEDING){
      break;
    }
  }

  last_lib_err_ = err;

  return p_ret;
}



static bool isAddrInRange(uint16_t addr, uint16_t length,
  uint16_t range_addr, uint16_t range_length)
{
  return (addr >= range_addr && addr+length <= range_addr+range_length) ? true:false;
}

static bool isAddrInOtherItem(uint16_t start_addr, uint16_t length,
  uint16_t other_start_addr, uint16_t other_length)
{
  bool ret = false;
  uint16_t addr_end = start_addr + length;
  uint16_t other_addr_end = other_start_addr + other_length;

  // The start address of the item is in the range of another item address.
  if (start_addr >= other_start_addr && start_addr < other_addr_end){ 
    ret = true;
  // The last address of an item is in the range of another item address.    
  }else if (addr_end > other_start_addr && addr_end <= other_addr_end){ 
    ret = true;
  }  

  return ret;
}
