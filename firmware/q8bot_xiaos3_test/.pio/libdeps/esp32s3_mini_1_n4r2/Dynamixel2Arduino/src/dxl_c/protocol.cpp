#include "protocol.h"

#if !defined(ESP_PLATFORM) && !defined(ARDUINO_ARCH_MBED_PORTENTA) && !defined(ARDUINO_ARCH_SAMD)
  #include <avr/pgmspace.h>
#endif

#if !defined(PROGMEM)
  #define PROGMEM
#endif

#if !defined(pgm_read_word_near)
  #define pgm_read_word_near(x) (*(uint16_t*)(x))
#endif

using namespace DYNAMIXEL; 

// 2.0 Protocol
#define DXL2_0_PACKET_IDX_HEADER_1        0
#define DXL2_0_PACKET_IDX_HEADER_2        1
#define DXL2_0_PACKET_IDX_HEADER_3        2
#define DXL2_0_PACKET_IDX_RESERVED        3
#define DXL2_0_PACKET_IDX_ID              4
#define DXL2_0_PACKET_IDX_LENGTH_L        5
#define DXL2_0_PACKET_IDX_LENGTH_H        6
#define DXL2_0_PACKET_IDX_INST            7
#define DXL2_0_PACKET_IDX_ERROR           8
#define DXL2_0_PACKET_IDX_INST_PARAM      8
#define DXL2_0_PACKET_IDX_STATUS_PARAM    9

// 1.0 Protocol
#define DXL1_0_PACKET_IDX_HEADER_1        0
#define DXL1_0_PACKET_IDX_HEADER_2        1
#define DXL1_0_PACKET_IDX_ID              2
#define DXL1_0_PACKET_IDX_LENGTH          3
#define DXL1_0_PACKET_IDX_INST            4
#define DXL1_0_PACKET_IDX_ERROR           4
#define DXL1_0_PACKET_IDX_PARAM           5


//-- Internal Variables
//


//-- External Variables
//


//-- Internal Functions
//
static DXLLibErrorCode_t parse_dxl1_0_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data);
static DXLLibErrorCode_t parse_dxl2_0_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data);
static DXLLibErrorCode_t fast_parse_dxl2_0_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data, InfoSyncReadInst_t *sync_read, InfoBulkReadInst_t *bulk_read);

static DXLLibErrorCode_t add_param_to_dxl1_0_packet(InfoToMakeDXLPacket_t* p_make_packet, uint8_t *p_param, uint16_t param_len);
static DXLLibErrorCode_t add_param_to_dxl2_0_packet(InfoToMakeDXLPacket_t* p_make_packet, uint8_t *p_param, uint16_t param_len);

static DXLLibErrorCode_t end_make_dxl1_0_packet(InfoToMakeDXLPacket_t* p_make_packet);
static DXLLibErrorCode_t end_make_dxl2_0_packet(InfoToMakeDXLPacket_t* p_make_packet);

static void update_dxl_crc(uint16_t *p_crc_cur, uint8_t recv_data);

//-- External Functions
//
DXLLibErrorCode_t begin_make_dxl_packet(InfoToMakeDXLPacket_t* p_make_packet, 
  uint8_t id, uint8_t protocol_ver, uint8_t inst_idx, uint8_t err_idx, 
  uint8_t* p_packet_buf, uint16_t packet_buf_capacity)
{
  if(p_packet_buf == NULL){
    return DXL_LIB_ERROR_NULLPTR;
  }
  
  if(protocol_ver == 2){
    if(id == 0xFD && id == 0xFF){ //http://emanual.robotis.com/docs/en/dxl/protocol2/#packet-id
      return DXL_LIB_ERROR_INVAILD_ID;
    }
    if(packet_buf_capacity < 11){ //http://emanual.robotis.com/docs/en/dxl/protocol2/#status-packet
      return DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
    }
  }else if(protocol_ver == 1){
    if(id == 0xFF){ //http://emanual.robotis.com/docs/en/dxl/protocol1/#packet-id
      return DXL_LIB_ERROR_INVAILD_ID;
    }
    if(packet_buf_capacity < 6){ //http://emanual.robotis.com/docs/en/dxl/protocol1/#instruction-packet
      return DXL_LIB_ERROR_NOT_ENOUGH_BUFFER_SIZE;
    }    
  }else{
    return DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;
  }

  p_make_packet->id = id;
  p_make_packet->protocol_ver = protocol_ver;
  p_make_packet->inst_idx = inst_idx;
  p_make_packet->err_idx = err_idx;
  p_make_packet->p_packet_buf = p_packet_buf;
  p_make_packet->packet_buf_capacity = packet_buf_capacity;
  p_make_packet->param_length = 0;
  p_make_packet->generated_packet_length = 0;
  p_make_packet->is_init = true;

  return DXL_LIB_OK;
}


DXLLibErrorCode_t add_param_to_dxl_packet(InfoToMakeDXLPacket_t* p_make_packet, uint8_t *p_param, uint16_t param_len)
{
  DXLLibErrorCode_t ret;

  if(p_make_packet == NULL){
    return DXL_LIB_ERROR_NULLPTR; 
  }else if(p_make_packet->is_init == false){
    return DXL_LIB_ERROR_NOT_INITIALIZED;
  }

  if(p_make_packet->protocol_ver == 2){
    ret = add_param_to_dxl2_0_packet(p_make_packet, p_param, param_len);
  }else if(p_make_packet->protocol_ver == 1){
    ret = add_param_to_dxl1_0_packet(p_make_packet, p_param, param_len);
  }else{
    ret = DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;
  }

  return ret;
}

static DXLLibErrorCode_t add_param_to_dxl1_0_packet(InfoToMakeDXLPacket_t* p_make_packet, uint8_t *p_param, uint16_t param_len)
{
  uint8_t *p_param_buf;
  uint16_t i, generated_param_len;

  generated_param_len = p_make_packet->param_length;
  if(p_make_packet->packet_buf_capacity < generated_param_len+param_len+6 //6 = Header(2)+ID(1)+Length(1)+Instruction(1)+CheckSum(1) : http://emanual.robotis.com/docs/en/dxl/protocol1/#instruction-packet
  || generated_param_len+param_len > 255-2){ //Length Frame Range(1byte) = Instruction(1)+Param(N)+CheckSum(1) : http://emanual.robotis.com/docs/en/dxl/protocol1/#length
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  p_param_buf = &p_make_packet->p_packet_buf[DXL1_0_PACKET_IDX_PARAM];

  for (i=0; i<param_len; i++)
  {
    p_param_buf[generated_param_len++] = p_param[i];
  }

  p_make_packet->param_length = generated_param_len;

  return DXL_LIB_OK;
}

static DXLLibErrorCode_t add_param_to_dxl2_0_packet(InfoToMakeDXLPacket_t* p_make_packet, uint8_t *p_param, uint16_t param_len)
{
  uint8_t *p_packet, *p_param_buf;
  uint16_t i, generated_param_len;

  generated_param_len = p_make_packet->param_length;
  p_packet = p_make_packet->p_packet_buf;
  if(p_make_packet->inst_idx == DXL_INST_STATUS){
    if(p_make_packet->packet_buf_capacity < generated_param_len+param_len+11){
      return DXL_LIB_ERROR_BUFFER_OVERFLOW;
    }
    p_param_buf = &p_packet[DXL2_0_PACKET_IDX_STATUS_PARAM];

    for (i=0; i<param_len; i++)
    {
      p_param_buf[generated_param_len++] = p_param[i];
        
      //Add byte stuffing (http://emanual.robotis.com/docs/en/dxl/protocol2/#processing-order-of-transmission)
      if(generated_param_len >= 3){
        if(p_param_buf[generated_param_len-3] == 0xFF 
        && p_param_buf[generated_param_len-2] == 0xFF 
        && p_param_buf[generated_param_len-1] == 0xFD){
          p_param_buf[generated_param_len++] = 0xFD;
        }
      }else if(generated_param_len == 2){
        if(p_packet[DXL2_0_PACKET_IDX_ERROR] == 0xFF
        && p_param_buf[generated_param_len-2] == 0xFF
        && p_param_buf[generated_param_len-1] == 0xFD){
          p_param_buf[generated_param_len++] = 0xFD;
        }
      }else if(generated_param_len == 1){
        if(p_packet[DXL2_0_PACKET_IDX_INST] == 0xFF
        && p_packet[DXL2_0_PACKET_IDX_ERROR] == 0xFF
        && p_param_buf[generated_param_len-1] == 0xFD){
          p_param_buf[generated_param_len++] = 0xFD;
        }
      }

      if(p_make_packet->packet_buf_capacity < generated_param_len+11){
        return DXL_LIB_ERROR_BUFFER_OVERFLOW;
      }
    }
  }else{ //Instruction Packet
    if(p_make_packet->packet_buf_capacity < generated_param_len+param_len+10){
      return DXL_LIB_ERROR_BUFFER_OVERFLOW;
    }
    p_param_buf = &p_packet[DXL2_0_PACKET_IDX_INST_PARAM];

    for (i=0; i<param_len; i++)
    {
      p_param_buf[generated_param_len++] = p_param[i];
        
      //Add byte stuffing (http://emanual.robotis.com/docs/en/dxl/protocol2/#processing-order-of-transmission)
      if(generated_param_len >= 3){
        if(p_param_buf[generated_param_len-3] == 0xFF 
        && p_param_buf[generated_param_len-2] == 0xFF 
        && p_param_buf[generated_param_len-1] == 0xFD){
          p_param_buf[generated_param_len++] = 0xFD;
        }
      }else if(generated_param_len == 2){
        if(p_packet[DXL2_0_PACKET_IDX_INST] == 0xFF
        && p_param_buf[generated_param_len-2] == 0xFF
        && p_param_buf[generated_param_len-1] == 0xFD){
          p_param_buf[generated_param_len++] = 0xFD;
        }
      }

      if(p_make_packet->packet_buf_capacity < generated_param_len+10){
        return DXL_LIB_ERROR_BUFFER_OVERFLOW;
      }
    }
  }

  p_make_packet->param_length = generated_param_len;

  return DXL_LIB_OK;
}



DXLLibErrorCode_t end_make_dxl_packet(InfoToMakeDXLPacket_t* p_make_packet)
{
  DXLLibErrorCode_t ret;

  if(p_make_packet == NULL){
    return DXL_LIB_ERROR_NULLPTR;
  }
  if(p_make_packet->is_init == false){
    return DXL_LIB_ERROR_NOT_INITIALIZED;
  } 
  if(p_make_packet->p_packet_buf == NULL){
    return DXL_LIB_ERROR_NULLPTR;
  }

  if(p_make_packet->protocol_ver == 2){
    ret = end_make_dxl2_0_packet(p_make_packet);
  }else if(p_make_packet->protocol_ver == 1){
    ret = end_make_dxl1_0_packet(p_make_packet);
  }else{
    ret = DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;
  }

  return ret;
}

static DXLLibErrorCode_t end_make_dxl1_0_packet(InfoToMakeDXLPacket_t* p_make_packet)
{
  uint8_t *p_packet;
  uint8_t check_sum = 0;
  uint16_t i, generated_packet_len = 0;

  if(p_make_packet->packet_buf_capacity < p_make_packet->param_length+6 //6 = Header(2)+ID(1)+Length(1)+Instruction(1)+CheckSum(1) : http://emanual.robotis.com/docs/en/dxl/protocol1/#instruction-packet
  || p_make_packet->param_length > 255-2){ //Length Frame Range(1byte) = Instruction(1)+Param(N)+CheckSum(1) : http://emanual.robotis.com/docs/en/dxl/protocol1/#length
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  p_packet = p_make_packet->p_packet_buf;

  p_packet[generated_packet_len++] = 0xFF;
  p_packet[generated_packet_len++] = 0xFF;
  p_packet[generated_packet_len++] = p_make_packet->id;
  p_packet[generated_packet_len++] = p_make_packet->param_length+2; //2 = Instruction(1)+CheckSum(1)
  if(p_make_packet->inst_idx == DXL_INST_STATUS){
    p_packet[generated_packet_len++] = p_make_packet->err_idx;
  }else{
    p_packet[generated_packet_len++] = p_make_packet->inst_idx;
  }
  generated_packet_len += p_make_packet->param_length;

  for (i=DXL1_0_PACKET_IDX_ID; i<generated_packet_len; i++)
  {
    check_sum += p_packet[i];
  }
  
  p_packet[generated_packet_len++] = ~(check_sum);

  p_make_packet->generated_packet_length = generated_packet_len;

  return DXL_LIB_OK;
}

static DXLLibErrorCode_t end_make_dxl2_0_packet(InfoToMakeDXLPacket_t* p_make_packet)
{
  uint8_t *p_packet;
  uint16_t i, generated_packet_len = 0, calculated_crc = 0;

  if(p_make_packet->inst_idx == DXL_INST_STATUS && p_make_packet->packet_buf_capacity < p_make_packet->param_length+11){
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }else if(p_make_packet->packet_buf_capacity < p_make_packet->param_length+10){
    return DXL_LIB_ERROR_BUFFER_OVERFLOW;
  }

  p_packet = p_make_packet->p_packet_buf;

  p_packet[generated_packet_len++] = 0xFF;
  p_packet[generated_packet_len++] = 0xFF;
  p_packet[generated_packet_len++] = 0xFD;
  p_packet[generated_packet_len++] = 0x00;
  p_packet[generated_packet_len++] = p_make_packet->id;
  if(p_make_packet->inst_idx == DXL_INST_STATUS){
    p_packet[generated_packet_len++] = (p_make_packet->param_length+4) >> 0; //4 = Instruction(1)+Error(1)+CRC(2)
    p_packet[generated_packet_len++] = (p_make_packet->param_length+4) >> 8; //4 = Instruction(1)+Error(1)+CRC(2)
    p_packet[generated_packet_len++] = p_make_packet->inst_idx;
    p_packet[generated_packet_len++] = p_make_packet->err_idx;
  }else{
    p_packet[generated_packet_len++] = (p_make_packet->param_length+3) >> 0; //3 = Instruction(1)+CRC(2)
    p_packet[generated_packet_len++] = (p_make_packet->param_length+3) >> 8; //3 = Instruction(1)+CRC(2)
    p_packet[generated_packet_len++] = p_make_packet->inst_idx;
  }
  generated_packet_len += p_make_packet->param_length;

  for (i=0; i<generated_packet_len; i++)
  {
    update_dxl_crc(&calculated_crc, p_packet[i]);
  }

  p_packet[generated_packet_len++] = calculated_crc >> 0;
  p_packet[generated_packet_len++] = calculated_crc >> 8;

  p_make_packet->generated_packet_length = generated_packet_len;  

  return DXL_LIB_OK;
}

DXLLibErrorCode_t begin_parse_dxl_packet(InfoToParseDXLPacket_t* p_parse_packet, 
  uint8_t protocol_ver, uint8_t* p_param_buf, uint16_t param_buf_capacity)
{
  if(param_buf_capacity > 0 && p_param_buf == NULL){
    return DXL_LIB_ERROR_NULLPTR;
  }
  
  if(protocol_ver != 1 && protocol_ver != 2){
    return DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;
  }

  p_parse_packet->protocol_ver = protocol_ver;
  p_parse_packet->p_param_buf = p_param_buf;
  p_parse_packet->param_buf_capacity = param_buf_capacity;
  // p_parse_packet->recv_param_len = 0;
  // p_parse_packet->parse_state = 0;
  p_parse_packet->is_init = true;

  return DXL_LIB_OK;
}

DXLLibErrorCode_t fast_begin_parse_dxl_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t protocol_ver)
{
  if (protocol_ver != 2)
    return DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;

  p_parse_packet->protocol_ver = protocol_ver;
  p_parse_packet->p_param_buf = nullptr;
  p_parse_packet->param_buf_capacity = 0;
  p_parse_packet->is_init = true;

  return DXL_LIB_OK;
}

DXLLibErrorCode_t parse_dxl_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data)
{
  DXLLibErrorCode_t ret;

  if(p_parse_packet == NULL){
    return DXL_LIB_ERROR_NULLPTR;
  }else if(p_parse_packet->is_init == false){
    return DXL_LIB_ERROR_NOT_INITIALIZED;
  } 

  if(p_parse_packet->protocol_ver == 2){
    ret = parse_dxl2_0_packet(p_parse_packet, recv_data);
  }else if(p_parse_packet->protocol_ver == 1){
    ret = parse_dxl1_0_packet(p_parse_packet, recv_data);
  }else{
    ret = DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;
  }

  return ret;
}

DXLLibErrorCode_t fast_parse_dxl_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data,
                                        InfoSyncReadInst_t *sync_read, InfoBulkReadInst_t *bulk_read)
{
  if (p_parse_packet == NULL)
    return DXL_LIB_ERROR_NULLPTR;
  else if (p_parse_packet->is_init == false)
    return DXL_LIB_ERROR_NOT_INITIALIZED;
  else if (p_parse_packet->protocol_ver != 2)
    return DXL_LIB_ERROR_INVAILD_PROTOCOL_VERSION;
  else if ((nullptr == sync_read) && (nullptr == bulk_read))
    return DXL_LIB_ERROR_NULLPTR;

  return fast_parse_dxl2_0_packet(p_parse_packet, recv_data, sync_read, bulk_read);
}

static DXLLibErrorCode_t parse_dxl1_0_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data)
{
  DXLLibErrorCode_t ret = DXL_LIB_PROCEEDING;

  switch(p_parse_packet->parse_state)
  {
    case DXL1_0_PACKET_PARSING_STATE_IDLE:
      if(p_parse_packet->header_cnt >= 2){
        p_parse_packet->header_cnt = 0;
      }
      p_parse_packet->header[p_parse_packet->header_cnt++] = recv_data;
      if(p_parse_packet->header_cnt == 2){
        if(p_parse_packet->header[0] == 0xFF
        && p_parse_packet->header[1] == 0xFF){
          p_parse_packet->recv_param_len = 0;
          p_parse_packet->calculated_check_sum = 0;
          p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_ID;
        }else{
          p_parse_packet->header[0] = p_parse_packet->header[1];
          p_parse_packet->header[1] = 0;
          p_parse_packet->header_cnt--;
        }
      }
      break;

    case DXL1_0_PACKET_PARSING_STATE_ID:
      if(recv_data != 0xFF){ //http://emanual.robotis.com/docs/en/dxl/protocol1/#packet-id
        p_parse_packet->id = recv_data;
        p_parse_packet->calculated_check_sum += recv_data;
        p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_LENGTH;        
      }else{
        p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_IDLE;
      }
      break;

    case DXL1_0_PACKET_PARSING_STATE_LENGTH:
      if((uint16_t)recv_data > p_parse_packet->param_buf_capacity+2){ // 2 = Instruction(1) + CheckSum(1)
        ret = DXL_LIB_ERROR_BUFFER_OVERFLOW;
        p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_IDLE;
      }else if(recv_data < 2){ // 2 = Instruction(1) + CheckSum(1)
        ret = DXL_LIB_ERROR_LENGTH;
        p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_IDLE;
      }else{
        p_parse_packet->packet_len = recv_data;
        p_parse_packet->calculated_check_sum += recv_data;
        p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_INST_ERR;
      }
      break;

    case DXL1_0_PACKET_PARSING_STATE_INST_ERR:
      p_parse_packet->inst_idx = recv_data;
      p_parse_packet->err_idx = recv_data;
      p_parse_packet->calculated_check_sum += recv_data;
      p_parse_packet->recv_param_len = 0;
      if(p_parse_packet->packet_len == 2){ // 2 = Instruction(1) + CheckSum(1)
        p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_CHECK_SUM;
      }else{
        p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_PARAM;
        if(p_parse_packet->p_param_buf == NULL){
          ret = DXL_LIB_ERROR_NULLPTR;
          p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_IDLE;
        }
      }
      break;

    case DXL1_0_PACKET_PARSING_STATE_PARAM:
      p_parse_packet->p_param_buf[p_parse_packet->recv_param_len++] = recv_data;
      p_parse_packet->calculated_check_sum += recv_data;
      if(p_parse_packet->recv_param_len == p_parse_packet->packet_len-2){ // 2 = Instruction(1) + CheckSum(1)
        p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_CHECK_SUM;
      }
      break;

    case DXL1_0_PACKET_PARSING_STATE_CHECK_SUM:
      p_parse_packet->calculated_check_sum = ~(p_parse_packet->calculated_check_sum);
      p_parse_packet->recv_check_sum = recv_data;
      if(p_parse_packet->calculated_check_sum == p_parse_packet->recv_check_sum){
        ret = DXL_LIB_OK;
      }else{
        ret = DXL_LIB_ERROR_CHECK_SUM;
      }
      p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_IDLE;
      break;

    default:
      p_parse_packet->parse_state = DXL1_0_PACKET_PARSING_STATE_IDLE;
      break;
  }

  return ret;
}

static DXLLibErrorCode_t parse_dxl2_0_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data)
{
  DXLLibErrorCode_t ret = DXL_LIB_PROCEEDING;
  static uint16_t byte_stuffing_cnt = 0;  // static variable

  switch(p_parse_packet->parse_state)
  {
    case DXL2_0_PACKET_PARSING_STATE_IDLE:
      if(p_parse_packet->header_cnt >= 3){
        p_parse_packet->header_cnt = 0;
        byte_stuffing_cnt = 0;  // static variable initialization
      }
      p_parse_packet->header[p_parse_packet->header_cnt++] = recv_data;
      if(p_parse_packet->header_cnt == 3){
        if(p_parse_packet->header[0] == 0xFF
        && p_parse_packet->header[1] == 0xFF
        && p_parse_packet->header[2] == 0xFD){
          p_parse_packet->recv_param_len = 0;
          p_parse_packet->calculated_crc = 0;
          update_dxl_crc(&p_parse_packet->calculated_crc, 0xFF);
          update_dxl_crc(&p_parse_packet->calculated_crc, 0xFF);
          update_dxl_crc(&p_parse_packet->calculated_crc, 0xFD);          
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_RESERVED;
        }else{
          p_parse_packet->header[0] = p_parse_packet->header[1];
          p_parse_packet->header[1] = p_parse_packet->header[2];
          p_parse_packet->header_cnt--;
        }
      }
      break;

    case DXL2_0_PACKET_PARSING_STATE_RESERVED:
      if(recv_data != 0xFD){ // http://emanual.robotis.com/docs/en/dxl/protocol2/#reserved
        p_parse_packet->reserved = recv_data;
        update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_ID;
      }else{
        ret = DXL_LIB_ERROR_WRONG_PACKET;
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
      }
      break;

    case DXL2_0_PACKET_PARSING_STATE_ID:
      if(recv_data < 0xFD || recv_data == DXL_BROADCAST_ID){ //http://emanual.robotis.com/docs/en/dxl/protocol2/#packet-id
        p_parse_packet->id = recv_data;
        update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_LENGTH_L;
      }else{
        ret = DXL_LIB_ERROR_INVAILD_ID;
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
      }
      break;

    case DXL2_0_PACKET_PARSING_STATE_LENGTH_L:
      p_parse_packet->packet_len = recv_data;
      update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
      p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_LENGTH_H;
      break;

    case DXL2_0_PACKET_PARSING_STATE_LENGTH_H:
      p_parse_packet->packet_len |= recv_data<<8;
      update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
      if(p_parse_packet->packet_len < 3){ // 3 = Instruction(1)+CRC(2)
        ret = DXL_LIB_ERROR_LENGTH;
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
      }else{
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_INST;
      }
      break;

    case DXL2_0_PACKET_PARSING_STATE_INST:
      p_parse_packet->inst_idx = recv_data;
      update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
      p_parse_packet->recv_param_len = 0;
      p_parse_packet->err_idx = 0;
      if(recv_data == DXL_INST_STATUS){
        if(p_parse_packet->packet_len < 4){ // 4 = Instruction(1)+Error(1)+CRC(2)
          ret = DXL_LIB_ERROR_LENGTH;
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
        }else{
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_ERROR;
        }
      }else{
        if(p_parse_packet->packet_len == 3){ // 3 = Instruction(1)+CRC(2)
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_CRC_L;
        }else{
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_PARAM;
        }
      }
      break;

    case DXL2_0_PACKET_PARSING_STATE_ERROR:
      p_parse_packet->err_idx = recv_data;
      update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
      if(p_parse_packet->packet_len == 4){ // 4 = Instruction(1)+Error(1)+CRC(2)
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_CRC_L;
        break;
      }
      p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_PARAM;
      break;

    case DXL2_0_PACKET_PARSING_STATE_PARAM:
      if(p_parse_packet->p_param_buf == NULL){
        ret = DXL_LIB_ERROR_NULLPTR;
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
        break;
      }
      if(p_parse_packet->param_buf_capacity < p_parse_packet->recv_param_len) {
        ret = DXL_LIB_ERROR_BUFFER_OVERFLOW;
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
        break;
      }
      p_parse_packet->p_param_buf[p_parse_packet->recv_param_len++] = recv_data;
      update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);

      //Remove byte stuffing (http://emanual.robotis.com/docs/en/dxl/protocol2/#processing-order-of-reception)
      if(p_parse_packet->inst_idx == DXL_INST_STATUS){
        if(p_parse_packet->recv_param_len >= 4){
          if(p_parse_packet->p_param_buf[p_parse_packet->recv_param_len-4] == 0xFF
          && p_parse_packet->p_param_buf[p_parse_packet->recv_param_len-3] == 0xFF
          && p_parse_packet->p_param_buf[p_parse_packet->recv_param_len-2] == 0xFD
          && p_parse_packet->p_param_buf[p_parse_packet->recv_param_len-1] == 0xFD){
            p_parse_packet->recv_param_len--;
            byte_stuffing_cnt++;
          }
        }else if(p_parse_packet->recv_param_len == 3){
          if(p_parse_packet->err_idx == 0xFF
          && p_parse_packet->p_param_buf[0] == 0xFF
          && p_parse_packet->p_param_buf[1] == 0xFD
          && p_parse_packet->p_param_buf[2] == 0xFD){
            p_parse_packet->recv_param_len--;
            byte_stuffing_cnt++;
          }
        }else if(p_parse_packet->recv_param_len == 2){
          if(p_parse_packet->inst_idx == 0xFF
          && p_parse_packet->err_idx == 0xFF
          && p_parse_packet->p_param_buf[0] == 0xFD
          && p_parse_packet->p_param_buf[1] == 0xFD){
            p_parse_packet->recv_param_len--;
            byte_stuffing_cnt++;
          }
        }
        if(p_parse_packet->recv_param_len+byte_stuffing_cnt+4 == p_parse_packet->packet_len){ // 4 = Instruction(1)+Error(1)+CRC(2)
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_CRC_L;
        }
      }else{
        if(p_parse_packet->recv_param_len >= 4){
          if(p_parse_packet->p_param_buf[p_parse_packet->recv_param_len-4] == 0xFF
          && p_parse_packet->p_param_buf[p_parse_packet->recv_param_len-3] == 0xFF
          && p_parse_packet->p_param_buf[p_parse_packet->recv_param_len-2] == 0xFD
          && p_parse_packet->p_param_buf[p_parse_packet->recv_param_len-1] == 0xFD){
            p_parse_packet->recv_param_len--;
            byte_stuffing_cnt++;
          }
        }else if(p_parse_packet->recv_param_len == 3){
          if(p_parse_packet->inst_idx == 0xFF
          && p_parse_packet->p_param_buf[0] == 0xFF
          && p_parse_packet->p_param_buf[1] == 0xFD
          && p_parse_packet->p_param_buf[2] == 0xFD){
            p_parse_packet->recv_param_len--;
            byte_stuffing_cnt++;
          }
        }
        if(p_parse_packet->recv_param_len+byte_stuffing_cnt+3 == p_parse_packet->packet_len){ // 3 = Instruction(1)+CRC(2)
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_CRC_L;
        }
      }
      break;

    case DXL2_0_PACKET_PARSING_STATE_CRC_L:
      p_parse_packet->recv_crc = recv_data;
      p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_CRC_H;
      break;

    case DXL2_0_PACKET_PARSING_STATE_CRC_H:
      p_parse_packet->recv_crc |= recv_data<<8;
      if (p_parse_packet->calculated_crc == p_parse_packet->recv_crc){
        ret = DXL_LIB_OK;
      }else{
        ret = DXL_LIB_ERROR_CRC;
      }
      p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
      break;

    default:
      p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
      break;
  }

  return ret;  
}

//fast_parse_dxl2_0_packet
static DXLLibErrorCode_t fast_parse_dxl2_0_packet(InfoToParseDXLPacket_t* p_parse_packet, uint8_t recv_data,
                                                  InfoSyncReadInst_t *sync_read, InfoBulkReadInst_t *bulk_read)
{
  DXLLibErrorCode_t ret = DXL_LIB_PROCEEDING;

  switch(p_parse_packet->parse_state)
    {
      case DXL2_0_PACKET_PARSING_STATE_IDLE:
        if(p_parse_packet->header_cnt >= 3){
          p_parse_packet->header_cnt = 0;
        }
        p_parse_packet->header[p_parse_packet->header_cnt++] = recv_data;
        if(p_parse_packet->header_cnt == 3){
          if(p_parse_packet->header[0] == 0xFF
          && p_parse_packet->header[1] == 0xFF
          && p_parse_packet->header[2] == 0xFD){
            p_parse_packet->recv_param_len = 0;
            p_parse_packet->calculated_crc = 0;
            update_dxl_crc(&p_parse_packet->calculated_crc, 0xFF);
            update_dxl_crc(&p_parse_packet->calculated_crc, 0xFF);
            update_dxl_crc(&p_parse_packet->calculated_crc, 0xFD);          
            p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_RESERVED;
          }else{
            p_parse_packet->header[0] = p_parse_packet->header[1];
            p_parse_packet->header[1] = p_parse_packet->header[2];
            p_parse_packet->header_cnt--;
          }
        }
        break;

      case DXL2_0_PACKET_PARSING_STATE_RESERVED:
        if(recv_data != 0xFD){ // http://emanual.robotis.com/docs/en/dxl/protocol2/#reserved
          p_parse_packet->reserved = recv_data;
          update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_ID;
        }else{
          ret = DXL_LIB_ERROR_WRONG_PACKET;
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
        }
        break;

      case DXL2_0_PACKET_PARSING_STATE_ID:
        if(recv_data < 0xFD || recv_data == DXL_BROADCAST_ID){ //http://emanual.robotis.com/docs/en/dxl/protocol2/#packet-id
          p_parse_packet->id = recv_data;
          update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_LENGTH_L;
        }else{
          ret = DXL_LIB_ERROR_INVAILD_ID;
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
        }
        break;

      case DXL2_0_PACKET_PARSING_STATE_LENGTH_L:
        p_parse_packet->packet_len = recv_data;
        update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_LENGTH_H;
        break;

      case DXL2_0_PACKET_PARSING_STATE_LENGTH_H:
        p_parse_packet->packet_len |= recv_data<<8;
        update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
        if(p_parse_packet->packet_len < 3){ // 3 = Instruction(1)+CRC(2)
          ret = DXL_LIB_ERROR_LENGTH;
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
        }else{
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_INST;
        }
        break;

      case DXL2_0_PACKET_PARSING_STATE_INST:
        p_parse_packet->inst_idx = recv_data;
        update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
        p_parse_packet->err_idx = 0;
        p_parse_packet->recv_param_len = 0;

        if(recv_data == DXL_INST_STATUS){
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_ERROR;
          if(p_parse_packet->packet_len < 4){ // 4 = Instruction(1)+Error(1)+CRC(2)
            ret = DXL_LIB_ERROR_LENGTH;
            p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
          }else{
            p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_ERROR;
          }
        }else{
          if(p_parse_packet->packet_len == 3){ // 3 = Instruction(1)+CRC(2)
            p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_CRC_L;
          }else{
            p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_PARAM;
          }
        }
        break;

      case DXL2_0_PACKET_PARSING_STATE_ERROR:
        p_parse_packet->err_idx = recv_data;
        update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
        if(p_parse_packet->packet_len == 4){ // 4 = Instruction(1)+Error(1)+CRC(2)
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_CRC_L;
          break;
        }
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_PARAM;

        // p_parse_packet->fast_param_state = DXL_FAST_PARAM_ERROR;
        p_parse_packet->xel_index = 0;
        p_parse_packet->buf_index = 0;
        if (nullptr != sync_read)
          sync_read->p_xels[0].error = recv_data;
        else if (nullptr != bulk_read)
          bulk_read->p_xels[0].error = recv_data;
        p_parse_packet->fast_param_state = DXL_FAST_PARAM_ID;
        break;

      case DXL2_0_PACKET_PARSING_STATE_PARAM:
        update_dxl_crc(&p_parse_packet->calculated_crc, recv_data);
        // https://emanual.robotis.com/docs/en/dxl/protocol2/#parameter
        if (nullptr != sync_read) {
          if (DXL_FAST_PARAM_ERROR == p_parse_packet->fast_param_state) {
            sync_read->p_xels[p_parse_packet->xel_index].error = recv_data;
            p_parse_packet->fast_param_state = DXL_FAST_PARAM_ID;
          } else if (DXL_FAST_PARAM_ID == p_parse_packet->fast_param_state) {
            sync_read->p_xels[p_parse_packet->xel_index].id = recv_data;
            p_parse_packet->fast_param_state = DXL_FAST_PARAM_DATA;
          } else if (DXL_FAST_PARAM_DATA == p_parse_packet->fast_param_state) {
            sync_read->p_xels[p_parse_packet->xel_index].p_recv_buf[p_parse_packet->buf_index] = recv_data;
            p_parse_packet->buf_index++;
            if (sync_read->addr_length == p_parse_packet->buf_index)
              p_parse_packet->fast_param_state = DXL_FAST_PARAM_CRC_L;
          } else if (DXL_FAST_PARAM_CRC_L == p_parse_packet->fast_param_state) {
            p_parse_packet->fast_param_state = DXL_FAST_PARAM_CRC_H;
          } else if (DXL_FAST_PARAM_CRC_H == p_parse_packet->fast_param_state) {
            p_parse_packet->xel_index++;
            p_parse_packet->buf_index = 0;
            p_parse_packet->fast_param_state = DXL_FAST_PARAM_ERROR;
          }
        } else if (nullptr != bulk_read) {
          if (DXL_FAST_PARAM_ERROR == p_parse_packet->fast_param_state) {
            bulk_read->p_xels[p_parse_packet->xel_index].error = recv_data;
            p_parse_packet->fast_param_state = DXL_FAST_PARAM_ID;
          } else if (DXL_FAST_PARAM_ID == p_parse_packet->fast_param_state) {
            bulk_read->p_xels[p_parse_packet->xel_index].id = recv_data;
            p_parse_packet->fast_param_state = DXL_FAST_PARAM_DATA;
          } else if (DXL_FAST_PARAM_DATA == p_parse_packet->fast_param_state) {
            bulk_read->p_xels[p_parse_packet->xel_index].p_recv_buf[p_parse_packet->buf_index] = recv_data;
            p_parse_packet->buf_index++;
            if (bulk_read->p_xels[p_parse_packet->xel_index].addr_length == p_parse_packet->buf_index)
              p_parse_packet->fast_param_state = DXL_FAST_PARAM_CRC_L;
          } else if (DXL_FAST_PARAM_CRC_L == p_parse_packet->fast_param_state) {
            p_parse_packet->fast_param_state = DXL_FAST_PARAM_CRC_H;
          } else if (DXL_FAST_PARAM_CRC_H == p_parse_packet->fast_param_state) {
            p_parse_packet->xel_index++;
            p_parse_packet->buf_index = 0;
            p_parse_packet->fast_param_state = DXL_FAST_PARAM_ERROR;
          }
        }
        p_parse_packet->recv_param_len += 1;
        if (p_parse_packet->recv_param_len == p_parse_packet->param_length)
          p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_CRC_L;
        break;

      case DXL2_0_PACKET_PARSING_STATE_CRC_L:
        p_parse_packet->recv_crc = recv_data;
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_CRC_H;
        break;

      case DXL2_0_PACKET_PARSING_STATE_CRC_H:
        p_parse_packet->recv_crc |= recv_data<<8;
        if (p_parse_packet->calculated_crc == p_parse_packet->recv_crc){
          ret = DXL_LIB_OK;
        }else{
          ret = DXL_LIB_ERROR_CRC;
        }
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
        break;

      default:
        p_parse_packet->parse_state = DXL2_0_PACKET_PARSING_STATE_IDLE;
        break;
    }
  return ret;  
}


const unsigned short crc_table[256] PROGMEM = { 0x0000,
  0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
  0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
  0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
  0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
  0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
  0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
  0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
  0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
  0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
  0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
  0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
  0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
  0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
  0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
  0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
  0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
  0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
  0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
  0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
  0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
  0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
  0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
  0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
  0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
  0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
  0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
  0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
  0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
  0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
  0x820D, 0x8207, 0x0202 };

static void update_dxl_crc(uint16_t *p_crc_cur, uint8_t recv_data)
{
  uint16_t crc;
  uint16_t i;

  crc = *p_crc_cur;

  i = ((unsigned short)(crc >> 8) ^ recv_data) & 0xFF;
  *p_crc_cur = (crc << 8) ^ pgm_read_word_near(crc_table + i);
}