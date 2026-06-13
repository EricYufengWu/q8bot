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

#define MAX_BAUD  5
const uint32_t baud[MAX_BAUD] = {57600, 115200, 1000000, 2000000, 3000000};
#define INVALID_ID 253

struct DxlList
{
  uint8_t dxl_id;
  uint32_t dxl_baudrate;
  uint8_t dxl_protocol;
};

//This namespace is required to use Control table item names
using namespace ControlTableItem;   

uint8_t itemList[] = {ID, BAUD_RATE, PROTOCOL_VERSION, MODEL_NUMBER, FIRMWARE_VERSION, RETURN_DELAY_TIME, DRIVE_MODE, OPERATING_MODE, SECONDARY_ID, HOMING_OFFSET, MOVING_THRESHOLD, TEMPERATURE_LIMIT, MAX_VOLTAGE_LIMIT, MIN_VOLTAGE_LIMIT, PWM_LIMIT, CURRENT_LIMIT, VELOCITY_LIMIT, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT, SHUTDOWN};
String itemListStr[] = {
  "ID                ",
  "BAUD RATE         ",
  "PROTOCOL VERSION  ",
  "MODEL NUMBER      ",
  "FIRMWARE VERSION  ",
  "RETURN DELAY TIME ",
  "DRIVE MODE        ",
  "OPERATING MODE    ",
  "SECONDARY ID      ",
  "HOMING OFFSET     ",
  "MOVING THRESHOLD  ",
  "TEMPERATURE LIMIT ",
  "MAX VOLTAGE LIMIT ",
  "MIN VOLTAGE LIMIT ",
  "PWM LIMIT         ",
  "CURRENT LIMIT     ",
  "VELOCITY LIMIT    ",
  "MAX POSITION LIMIT",
  "MIN POSITION LIMIT",
  "SHUTDOWN          "};

struct DxlList DXLArray[10];
int8_t found_dynamixel = 0;
int MasterString = 0;
uint8_t saved_index_a = 0;
uint8_t saved_index_b = 0;
uint8_t dxl_a = 0;
uint8_t dxl_b = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() 
{
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);   //set debugging port baudrate to 115200bps
  while(!DEBUG_SERIAL);         //Wait until the serial port is opened

  DEBUG_SERIAL.println(F("//*********** DYNAMIXEL X Series EEPROM Copy ***********//"));
  DEBUG_SERIAL.println(F("//*** This example is written for XM/XH Series Only ****//"));
  DEBUG_SERIAL.println(F("//********* Up to 10 DYNAMIXEL can be detected *********//\n"));
  scanDynamixel();
}

bool scanDynamixel()
{
  memset(DXLArray, 0, sizeof(DXLArray));
  found_dynamixel = 0;
  bool result = false;
  int8_t index = 0;

  for(uint8_t protocol = 1; protocol < 3; protocol++) 
  {
    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
    dxl.setPortProtocolVersion((float)protocol);
    DEBUG_SERIAL.print(F("SCAN PROTOCOL "));
    DEBUG_SERIAL.println(protocol);

    for(index = 0; index < MAX_BAUD; index++) 
    {
      // Set Port baudrate.
      DEBUG_SERIAL.print(F("BAUDRATE "));
      DEBUG_SERIAL.println(baud[index]);
      dxl.begin(baud[index]);
      for(uint8_t id = 0; id < DXL_BROADCAST_ID; id++) 
      {
        //iterate until all ID in each baudrate is scanned.
        if(dxl.ping(id)) 
        {
          DEBUG_SERIAL.print(F("\tID : "));
          DEBUG_SERIAL.print(id);
          DEBUG_SERIAL.print(F(", Model Number: "));
          DEBUG_SERIAL.println(dxl.getModelNumber(id));
          // Save detected DYNAMIXEL info (ID & baudrate) to global variable to access from copyEEPROM()
          DXLArray[found_dynamixel] = {id, baud[index], protocol};
          found_dynamixel++;
          result = true;
        }
      }
    }
  }
  DEBUG_SERIAL.print(F("\nTotal "));
  DEBUG_SERIAL.print(found_dynamixel);
  DEBUG_SERIAL.println(F(" DYNAMIXEL(s) found!"));

  return result;
}

uint8_t validIdCheck(uint8_t id)
{
  for(int i = 0; i <= found_dynamixel; i++)
  {
    if(DXLArray[i].dxl_id == id)
    {
      return i;
    }
  }
  return INVALID_ID;
}

void safetyCheck(uint8_t id_a, uint8_t id_b)
{
  uint16_t modelNumberA, modelNumberB;
  uint8_t fwVersionA, fwVersionB;
  saved_index_a = validIdCheck(id_a);
  saved_index_b = validIdCheck(id_b);

  //Torque off all DYNAMIXEL
  for(int i=0 ; i < found_dynamixel; i++)
  {
    dxl.begin(DXLArray[i].dxl_baudrate);
    dxl.torqueOff(DXLArray[i].dxl_id);
  }
    
  // Check Model Number, F/W version match
  dxl.setPortProtocolVersion((float)DXLArray[validIdCheck(id_a)].dxl_protocol);
  dxl.begin(DXLArray[validIdCheck(id_a)].dxl_baudrate);
  modelNumberA = dxl.getModelNumber(id_a);
  fwVersionA = dxl.readControlTableItem(FIRMWARE_VERSION, id_a);
  dxl.setPortProtocolVersion((float)DXLArray[validIdCheck(id_b)].dxl_protocol);
  dxl.begin(DXLArray[validIdCheck(id_b)].dxl_baudrate);
  modelNumberB = dxl.getModelNumber(id_b);
  fwVersionB = dxl.readControlTableItem(FIRMWARE_VERSION, id_b);

  if(modelNumberA != modelNumberB)
  {
    DEBUG_SERIAL.println(F("[ERROR] Please select identical DYNAMIXEL series!"));
    DEBUG_SERIAL.print(F("ID "));
    DEBUG_SERIAL.print(id_a);
    DEBUG_SERIAL.print(F(" model number : "));
    DEBUG_SERIAL.println(modelNumberA);
    DEBUG_SERIAL.print(F("ID "));
    DEBUG_SERIAL.print(id_b);
    DEBUG_SERIAL.print(F(" model number : "));
    DEBUG_SERIAL.println(modelNumberB);
  }
  else
  {
    if(fwVersionA != fwVersionB)
    {
      DEBUG_SERIAL.println(F("[ERROR] DYNAMIXEL Firmware version do not match!"));
      DEBUG_SERIAL.print(F("ID "));
      DEBUG_SERIAL.print(id_a);
      DEBUG_SERIAL.print(F(" firmware version : "));
      DEBUG_SERIAL.println(fwVersionA);
      DEBUG_SERIAL.print(F("ID "));
      DEBUG_SERIAL.print(id_b);
      DEBUG_SERIAL.print(F(" firmware version : "));
      DEBUG_SERIAL.println(fwVersionB);
    }
    // Compare EEPROM Data
    else
    {
      if((saved_index_a != INVALID_ID) && (saved_index_b != INVALID_ID))
      {
        for (uint8_t i = 0; i < sizeof(itemList); i++)
        {
          dxl.setPortProtocolVersion((float)DXLArray[saved_index_a].dxl_protocol);
          dxl.begin(DXLArray[saved_index_a].dxl_baudrate);
          DEBUG_SERIAL.print(itemListStr[i]);
          DEBUG_SERIAL.print(": ");
          if(i == 9) {DEBUG_SERIAL.print(dxl.readControlTableItem(itemList[i], id_a));}
          else if(i == 0 || i == 5 || i == 8) {DEBUG_SERIAL.print((uint8_t)dxl.readControlTableItem(itemList[i], id_a));}
          else {DEBUG_SERIAL.print((uint32_t)dxl.readControlTableItem(itemList[i], id_a));}
          dxl.setPortProtocolVersion((float)DXLArray[saved_index_b].dxl_protocol);
          dxl.begin(DXLArray[saved_index_b].dxl_baudrate);
          DEBUG_SERIAL.print("    \t");
          if(i == 9) {DEBUG_SERIAL.println(dxl.readControlTableItem(itemList[i], id_b));}
          else if(i == 0 || i == 5 || i == 8) {DEBUG_SERIAL.println((uint8_t)dxl.readControlTableItem(itemList[i], id_b));}
          else {DEBUG_SERIAL.println((uint32_t)dxl.readControlTableItem(itemList[i], id_b));}
        }
      }
    }
  }
}

void copyEEPROM(uint8_t start_index)
{
  for (uint8_t i = start_index; i < sizeof(itemList); i++)
  {
    int32_t data = 0;
    dxl.setPortProtocolVersion((float)DXLArray[saved_index_a].dxl_protocol);
    dxl.begin(DXLArray[saved_index_a].dxl_baudrate);
    
    DEBUG_SERIAL.print(F("Copying "));
    DEBUG_SERIAL.print(itemListStr[i]);
    
    data = dxl.readControlTableItem(itemList[i], DXLArray[saved_index_a].dxl_id);

    dxl.setPortProtocolVersion((float)DXLArray[saved_index_b].dxl_protocol);
    dxl.begin(DXLArray[saved_index_b].dxl_baudrate);

    if(dxl.writeControlTableItem(itemList[i], DXLArray[saved_index_b].dxl_id, data))
    {
      DEBUG_SERIAL.println(F(" Succeeded."));
    }
    else 
    {
      DEBUG_SERIAL.println(F(" Failed."));
    }
  }
}

bool copyBaudProtocol()
{
  int32_t baudrate, protocol = 0;
  bool ret = false;

  dxl.setPortProtocolVersion((float)DXLArray[saved_index_a].dxl_protocol);
  dxl.begin(DXLArray[saved_index_a].dxl_baudrate);
  
  baudrate = dxl.readControlTableItem(BAUD_RATE, DXLArray[saved_index_a].dxl_id);
  protocol = dxl.readControlTableItem(PROTOCOL_VERSION, DXLArray[saved_index_a].dxl_id);

  dxl.setPortProtocolVersion((float)DXLArray[saved_index_b].dxl_protocol);
  dxl.begin(DXLArray[saved_index_b].dxl_baudrate);
  DEBUG_SERIAL.print(F("Copying Protocol Version"));
  if(dxl.writeControlTableItem(PROTOCOL_VERSION, DXLArray[saved_index_b].dxl_id, protocol))
  {
    DEBUG_SERIAL.println(F("\tSucceeded."));
    ret = true;
  }
  else 
  {
    DEBUG_SERIAL.println(F("\tFailed."));
  }

  dxl.setPortProtocolVersion((float)DXLArray[saved_index_a].dxl_protocol);
  DEBUG_SERIAL.print(F("Copying Baud Rate"));
  if(dxl.writeControlTableItem(BAUD_RATE, DXLArray[saved_index_b].dxl_id, baudrate))
  {
    DEBUG_SERIAL.println(F("\tSucceeded."));
    ret = true;
  }
  else 
  {
    DEBUG_SERIAL.println(F("\tFailed."));
  }

  if(ret==true)
  {
    DEBUG_SERIAL.println(F("Also copy ID? [Y/N]"));
    DEBUG_SERIAL.println(F("**WARNING** Copying ID will cause communication collision!"));
    while (!DEBUG_SERIAL.available());
    MasterString = DEBUG_SERIAL.read();
    DEBUG_SERIAL.read();  // This is called just to reset the Serial.available();

    if(MasterString == 'y' || MasterString == 'Y')
    {
      DEBUG_SERIAL.print(F("Copying ID"));
      if(!copyId())
      {
        DEBUG_SERIAL.println(F("\tFailed."));
      }
      else
      {
        DEBUG_SERIAL.println(F("\tSucceeded."));
      }
    }
    else
    {
      DEBUG_SERIAL.println(F("Exit without copying ID."));
    }
  }
  return ret;
}

bool copyId()
{
  int32_t id = 0;

  dxl.setPortProtocolVersion((float)DXLArray[saved_index_a].dxl_protocol);
  dxl.begin(DXLArray[saved_index_a].dxl_baudrate);
  id = dxl.readControlTableItem(ID, DXLArray[saved_index_a].dxl_id);
  if(dxl.writeControlTableItem(ID, DXLArray[saved_index_b].dxl_id, id))
  {
    return true;
  }
  else 
  {
    return false;
  }
}

void loop() 
{
  // Rescan if none of DYNAMIXEL detected
  if(found_dynamixel == 0)
  {
    scanDynamixel();
  }
  DEBUG_SERIAL.print(F("\n[STEP 1] Please enter the DYNAMIXEL ID to read:  "));

  while (!DEBUG_SERIAL.available());
  MasterString = DEBUG_SERIAL.parseInt();
  DEBUG_SERIAL.read();  // This is called just to reset the Serial.available();
  DEBUG_SERIAL.println(MasterString);

  if((MasterString < 0) || (MasterString > INVALID_ID)) 
  {
    DEBUG_SERIAL.println(F("\tERROR : ID out of range!!!\n"));
  }
  else 
  {
    dxl_a = (uint8_t)MasterString;

    if(validIdCheck(dxl_a) == INVALID_ID)
    {
      DEBUG_SERIAL.println(F("\tERROR : Invalid ID!!!\n"));
    }
    else 
    {
      // compareEEPROM(dxl_a);
      DEBUG_SERIAL.print(F("[STEP 2] Please enter the DYNAMIXEL ID to write:  "));

      while (!DEBUG_SERIAL.available());
      MasterString = DEBUG_SERIAL.parseInt();
      DEBUG_SERIAL.read();  // This is called just to reset the Serial.available();
      DEBUG_SERIAL.println(MasterString);
      if((MasterString < 0) || (MasterString > INVALID_ID)) 
      {
        DEBUG_SERIAL.println(F("\tERROR : ID out of range!!!\n"));
      }
      else 
      {
        dxl_b = (uint8_t)MasterString;
        if((validIdCheck(dxl_b) == INVALID_ID) || (dxl_b == dxl_a))
        {
          DEBUG_SERIAL.println(F("\tERROR : Invalid ID!!!\n"));
        }
        else
        {
          safetyCheck(dxl_a, dxl_b);
          
          // Start copying EEPROM data
          if((saved_index_a != INVALID_ID) && (saved_index_b != INVALID_ID))
          {
            DEBUG_SERIAL.println(F("Start EEPROM copy? [Y/N/A]"));
            DEBUG_SERIAL.println(F("WARNING!!! [A] option will also copy ID, Baudrate, Protocol"));
            DEBUG_SERIAL.println(F("that will cause communication collision of connected DYNAMIXEL."));
            while (!DEBUG_SERIAL.available());
            MasterString = DEBUG_SERIAL.read();
            DEBUG_SERIAL.read();  // This is called just to reset the Serial.available();

            switch(MasterString)
            {
              case 'Y':
              case 'y':
                copyEEPROM(5);
                break;
              case 'N':
              case 'n':
                break;
              case 'A':
              case 'a':
                copyEEPROM(5);
                copyBaudProtocol();
                break;
              default:
                break;
            }
          }
        }
      }
    }
  }

  DEBUG_SERIAL.println(F("\nScan DYNAMIXEL again? [Y/N]"));

  while (!DEBUG_SERIAL.available());
  MasterString = DEBUG_SERIAL.read();
  DEBUG_SERIAL.read();  // This is called just to reset the Serial.available();

  if(MasterString == 'y' || MasterString == 'Y')
  {
    scanDynamixel();
  }
}
