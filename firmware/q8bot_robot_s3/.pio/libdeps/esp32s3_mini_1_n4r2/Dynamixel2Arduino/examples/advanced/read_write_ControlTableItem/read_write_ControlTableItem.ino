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

/* ControlTableItem (1.0 and 2.0)
  The naming convention for the ControlTable Item macro is as follows:
    From the name specified in the Control Table of each model(please refer to the e-manual (http://emanual.robotis.com)),
    the alphabet is replaced with capital letters
    and the space(" ") is replaced with an underscore(_).
    
  MODEL_NUMBER,
  MODEL_INFORMATION,
  FIRMWARE_VERSION,
  PROTOCOL_VERSION,
  ID,
  SECONDARY_ID,
  BAUD_RATE,
  DRIVE_MODE,
  CONTROL_MODE,
  OPERATING_MODE,
  CW_ANGLE_LIMIT,
  CCW_ANGLE_LIMIT,
  TEMPERATURE_LIMIT,
  MIN_VOLTAGE_LIMIT,
  MAX_VOLTAGE_LIMIT,
  PWM_LIMIT,
  CURRENT_LIMIT,
  VELOCITY_LIMIT,
  MAX_POSITION_LIMIT,
  MIN_POSITION_LIMIT,
  ACCELERATION_LIMIT,
  MAX_TORQUE,
  HOMING_OFFSET,
  MOVING_THRESHOLD,
  MULTI_TURN_OFFSET,
  RESOLUTION_DIVIDER,
  EXTERNAL_PORT_MODE_1,
  EXTERNAL_PORT_MODE_2,
  EXTERNAL_PORT_MODE_3,
  EXTERNAL_PORT_MODE_4,
  STATUS_RETURN_LEVEL,
  RETURN_DELAY_TIME,
  ALARM_LED,
  SHUTDOWN,

  TORQUE_ENABLE,
  LED,
  LED_RED,
  LED_GREEN,
  LED_BLUE,
  REGISTERED_INSTRUCTION,
  HARDWARE_ERROR_STATUS,
  VELOCITY_P_GAIN,
  VELOCITY_I_GAIN,
  POSITION_P_GAIN,
  POSITION_I_GAIN,
  POSITION_D_GAIN,
  FEEDFORWARD_1ST_GAIN,
  FEEDFORWARD_2ND_GAIN,
  P_GAIN,
  I_GAIN,
  D_GAIN,
  CW_COMPLIANCE_MARGIN,
  CCW_COMPLIANCE_MARGIN,
  CW_COMPLIANCE_SLOPE,
  CCW_COMPLIANCE_SLOPE,
  GOAL_PWM,
  GOAL_TORQUE,
  GOAL_CURRENT,
  GOAL_POSITION,
  GOAL_VELOCITY,
  GOAL_ACCELERATION,
  MOVING_SPEED,
  PRESENT_PWM,
  PRESENT_LOAD,
  PRESENT_SPEED,
  PRESENT_CURRENT,
  PRESENT_POSITION,
  PRESENT_VELOCITY,
  PRESENT_VOLTAGE,
  PRESENT_TEMPERATURE,
  TORQUE_LIMIT,
  REGISTERED,
  MOVING,
  LOCK,
  PUNCH,
  CURRENT,
  SENSED_CURRENT,
  REALTIME_TICK,
  TORQUE_CTRL_MODE_ENABLE,
  BUS_WATCHDOG,
  PROFILE_ACCELERATION,
  PROFILE_VELOCITY,
  MOVING_STATUS,
  VELOCITY_TRAJECTORY,
  POSITION_TRAJECTORY,
  PRESENT_INPUT_VOLTAGE,
  EXTERNAL_PORT_DATA_1,
  EXTERNAL_PORT_DATA_2,
  EXTERNAL_PORT_DATA_3,
  EXTERNAL_PORT_DATA_4,
*/

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

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;   

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.setPortProtocolVersion(2.0);
  dxl.begin(57600);
  dxl.scan();

  // Turn on torque
  dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 0);

  /* set velocity(speed) mode */
  // MX2.0, X serise
  dxl.writeControlTableItem(OPERATING_MODE, DXL_ID, 1);
  // AX~MX
  dxl.writeControlTableItem(CW_ANGLE_LIMIT, DXL_ID, 0);
  dxl.writeControlTableItem(CCW_ANGLE_LIMIT, DXL_ID, 0);
  // XL320
  dxl.writeControlTableItem(CONTROL_MODE, DXL_ID, 1);
  
  // Turn on torque
  dxl.writeControlTableItem(TORQUE_ENABLE, DXL_ID, 1);
}

void loop() {
  // put your main code here, to run repeatedly:

  /* Write & read velocity(speed) data */
  // MX2.0, X serise
  if(dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 200)){
    delay(500);
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_VELOCITY, DXL_ID));
  }
  // AX~MX & XL320
  if(dxl.writeControlTableItem(MOVING_SPEED, DXL_ID, 200)){
    delay(500);
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_SPEED, DXL_ID));
  }

  delay(2000);  

  // MX2.0, X serise
  if(dxl.writeControlTableItem(GOAL_VELOCITY, DXL_ID, 0)){
    delay(500);
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_VELOCITY, DXL_ID));
  }
  // AX~MX & XL320
  if(dxl.writeControlTableItem(MOVING_SPEED, DXL_ID, 0)){
    delay(500);
    DEBUG_SERIAL.println(dxl.readControlTableItem(PRESENT_SPEED, DXL_ID));
  }

  delay(2000);  
}