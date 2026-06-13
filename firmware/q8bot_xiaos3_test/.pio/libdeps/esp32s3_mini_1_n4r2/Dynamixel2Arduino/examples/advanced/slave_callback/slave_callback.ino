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
 

// Create a port object for DYNAMIXEL communication.
// The list of available classes is as follows.
// 1) DYNAMIXEL::SerialPortHandler     (HardwareSerial only)
//     -note: If you do not want to use half duplex communication, do not enter the second parameter.
// 2) DYNAMIXEL::USBSerialPortHandler  (Only USB port on each board)
DYNAMIXEL::SerialPortHandler dxl_port(DXL_SERIAL, DXL_DIR_PIN);

// Create a Slave object to communicate with the master.
const float DXL_PROTOCOL_VER_1_0 = 1.0;
const float DXL_PROTOCOL_VER_2_0 = 2.0;
const uint16_t DXL_MODEL_NUM = 0x5005; // Modify it to what you want.
DYNAMIXEL::Slave dxl(dxl_port, DXL_MODEL_NUM);

// Declare the address of the Slave control item you want 
// to register and the variable (size is also important)
// to store its data.
const uint16_t ADDR_CONTROL_ITEM_LED = 10;
const uint16_t ADDR_CONTROL_ITEM_ANALOG = 20;
// note: 'int' is not supported because its size varies by system architecture.
uint8_t control_item_led;
int16_t control_item_analog;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);

  // Speed setting for communication (not necessary for USB)
  dxl_port.begin(1000000);

  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER_2_0);
  dxl.setFirmwareVersion(1);
  dxl.setID(1);

  // Add control items.
  // Input the address(p1) and the variable(p2) to write/read the data as parameters.
  dxl.addControlItem(ADDR_CONTROL_ITEM_LED, control_item_led);
  dxl.addControlItem(ADDR_CONTROL_ITEM_ANALOG, control_item_analog);

  // Add interrupt callback functions to process read/write packet.
  dxl.setReadCallbackFunc(read_callback_func);
  dxl.setWriteCallbackFunc(write_callback_func);
}

void loop() {
  // put your main code here, to run repeatedly:

  // If there is a packet from the master, process it.
  if(dxl.processPacket() == false){
    DEBUG_SERIAL.print("Last lib err code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print("Last status packet err code: ");
    DEBUG_SERIAL.println(dxl.getLastStatusPacketError());
    DEBUG_SERIAL.println();
  }
}


// Function to update data according to master's read request.
// If you use dxl_err_code, you can send the packet's error code to the Master.
// See the DYNAMIXEL Protocol documentation for this. 
// (http://emanual.robotis.com/docs/kr/dxl/protocol2/)
void read_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)dxl_err_code, (void)arg;

  if(item_addr == ADDR_CONTROL_ITEM_ANALOG){
    control_item_analog = analogRead(A0);
  }
}

// Function to update data according to master write request.
// If you use dxl_err_code, you can send the packet's error code to the Master.
// See the DYNAMIXEL Protocol documentation for this. 
// (http://emanual.robotis.com/docs/kr/dxl/protocol2/)
void write_callback_func(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
{
  (void)dxl_err_code, (void)arg;

  if(item_addr == ADDR_CONTROL_ITEM_LED){
    digitalWrite(LED_BUILTIN, control_item_led);
  }
}
