#include <cstring>

#include "communications.h"
#include "helpers.h"
#include "systemParams.h"

Message::Message(unsigned char* data) {
  // Parse data
  cmd = (CommandType)data[0];
  
  payload_len = data[1];

  payload = std::unique_ptr<MessagePayload_t[]>(new MessagePayload_t[payload_len]);

  if (payload_len > 0) {
    memcpy(payload.get(), data + 2, payload_len * sizeof(MessagePayload_t));
  }
}

CommandType Message::GetCommand() {
  return cmd;
}

uint8_t Message::GetPayloadLen() {
  return payload_len;
}

int32_t* Message::GetPayloadInts() {
  return (int32_t*)(payload.get());
}

float* Message::GetPayloadFloats() {
  return (float*)(payload.get());
}