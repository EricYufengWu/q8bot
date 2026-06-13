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
 

const uint8_t DXL_ID = 1;
static uint8_t op_mode = OP_POSITION;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.scan();
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t pre_time_write, pre_time_read, pre_time_op_mode, pre_time_led;
  static float value = 0;
  static bool led_state, flag_op_changed = true;
  
  switch(op_mode)
  {
    case OP_POSITION:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          nextOperatingMode();
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalPosition(DXL_ID, value);
        value += 5;
      }     
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Position : ");
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID));
      }
      break;
    
    case OP_EXTENDED_POSITION:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          nextOperatingMode();
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalPosition(DXL_ID, value);
        value += 50;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Extended Position : ");
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID));
      }
      break;
      
    case OP_CURRENT_BASED_POSITION:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          nextOperatingMode();
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalPosition(DXL_ID, value);
        value += 50;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Current Based Position : ");
        DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID));
      }
      break;

    case OP_VELOCITY:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          nextOperatingMode();
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalVelocity(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Velocity : ");
        DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID));
      }    
      break;

    case OP_PWM:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          nextOperatingMode();
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalPWM(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present PWM : ");
        DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID));
      }    
      break;

    case OP_CURRENT:
      if(flag_op_changed){
        value = 0;
        dxl.torqueOff(DXL_ID);
        if(dxl.setOperatingMode(DXL_ID, op_mode) == false){
          nextOperatingMode();
          break;
        }
        dxl.torqueOn(DXL_ID);
        flag_op_changed = false;
      }

      if(millis() - pre_time_write >= 100) {    
        pre_time_write = millis();
        dxl.setGoalCurrent(DXL_ID, value);
        value += 2;
      }
      if(millis() - pre_time_read >= 50) {
        pre_time_read = millis();
        DEBUG_SERIAL.print("Present Current : ");
        DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID));
      }
      break;

    default:
      op_mode = OP_POSITION;
      break;
  }

  if(millis() - pre_time_op_mode >= (uint32_t)60*1000){
    pre_time_op_mode = millis();
    nextOperatingMode();
    flag_op_changed = true;
  }

  if(millis() - pre_time_led >= 500) {
    pre_time_led = millis();
    led_state == true ? dxl.ledOn(DXL_ID) : dxl.ledOff(DXL_ID);
    led_state = !led_state;
  }
}

void nextOperatingMode() {
  switch (op_mode) {
    case OP_CURRENT:
      op_mode = OP_VELOCITY;
      break;
    case OP_VELOCITY:
      op_mode = OP_POSITION;
      break;
    case OP_POSITION:
      op_mode = OP_EXTENDED_POSITION;
      break;
    case OP_EXTENDED_POSITION:
      op_mode = OP_CURRENT_BASED_POSITION;
      break;
    case OP_CURRENT_BASED_POSITION:
      op_mode = OP_PWM;
      break;
    case OP_PWM:
      op_mode = OP_CURRENT;
      break;
    default:
      op_mode = OP_POSITION;
  }
}