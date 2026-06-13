/*
   MAX1704X Arduino Library for MAX17043 and MAX17044 Fuel Gauge.

   Copyright © 2018-2022 Daniel Porrey. All Rights Reserved.
   https://github.com/porrey/max1704x

   This file is part of the MAX1704X Arduino Library.

   The MAX1704X Arduino Library is free software: you can redistribute
   it and/or modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation, either version 3 of the
   License, or (at your option) any later version.

   The MAX1704X Arduino Library is distributed in the hope that it
   will be useful, but WITHOUT ANY WARRANTY; without even the implied
   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
   the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with the MAX1704X Arduino Library. If not,
   see http://www.gnu.org/licenses/.
*/
#include <Wire.h>
#include <Arduino.h>

/*
   This stand alone sketch will attempt to find your MAX1704X
   device and dump the contents of the registers. It will also
   perform a calculation to show the current voltage as well as
   display the state of charge (SOC).
*/

#define REGISTER_VCELL      0x02
#define REGISTER_SOC        0x04
#define REGISTER_MODE       0x06
#define REGISTER_VERSION    0x08
#define REGISTER_CONFIG     0x0C
#define REGISTER_COMMAND    0xFE
#define RESET_COMMAND       0x5400
#define QUICKSTART_MODE     0x4000

//
// Macros
//
#define toLowByte(w)              ((uint8_t) ((w) & 0xff))
#define toHighByte(w)             ((uint8_t) ((w) >> 8))
#define toUint16(highB, lowB)     ((uint16_t)((uint8_t)highB << 8) + (uint8_t)lowB)

//
// The device address will be intialized
// by the device search.
//
uint16_t _deviceAddress = 0;

void setup()
{
  //
  // Initialize the serial interface.
  //
  Serial.begin(115200);
  delay(250);

  //
  // Wait for serial port to connect.
  //
  while (!Serial) {}
  Serial.println("Serial port initialized.\n");

  //
  // Initialize Wire.
  //
  Wire.begin();

  //
  // Find the first device using IC version 0x03.
  //
  Serial.println("Searching for a device...");
  _deviceAddress = findFirstDevice(0x03);

  if (_deviceAddress > 0)
  {
    Serial.print("A MAX1704x device was found at address 0x"); Serial.println(_deviceAddress, HEX);

    //
    // Issue a reset command and wait
    // for the device to be ready.
    //
    Serial.println("Resetting device...");
    writeRegister16(_deviceAddress, REGISTER_COMMAND, RESET_COMMAND);
    delay(250);

    //
    // Issue a quickstart command and wait
    // for the device to be ready.
    //
    Serial.println("Initiating quickstart mode...");
    writeRegister16(_deviceAddress, REGISTER_COMMAND, QUICKSTART_MODE);
    delay(125);

    //
    // Get the device data.
    //
    Serial.println("Reading device...");
    readDevice();
  }
  else
  {
    Serial.println("A MAX1704x device was not found!");
  }
}

//
// Read the data from the device.
//
void readDevice()
{
  uint8_t vcell_h = readRegister8(_deviceAddress, REGISTER_VCELL);
  uint8_t vcell_l = readRegister8(_deviceAddress, REGISTER_VCELL + 1);
  uint8_t soc_h = readRegister8(_deviceAddress, REGISTER_SOC);
  uint8_t soc_l = readRegister8(_deviceAddress, REGISTER_SOC + 1);
  uint8_t mode_h = readRegister8(_deviceAddress, REGISTER_MODE);
  uint8_t mode_l = readRegister8(_deviceAddress, REGISTER_MODE + 1);
  uint8_t ver_h = readRegister8(_deviceAddress, REGISTER_VERSION);
  uint8_t ver_l = readRegister8(_deviceAddress, REGISTER_VERSION + 1);
  uint8_t config_h = readRegister8(_deviceAddress, REGISTER_CONFIG);
  uint8_t config_l = readRegister8(_deviceAddress, REGISTER_CONFIG + 1);

  Serial.println();
  Serial.println("RAW VALUES");
  Serial.println("----------------------------------------");
  Serial.print("ADDRESS............: 0x"); Serial.print(_deviceAddress, HEX); Serial.print("\t("); Serial.print(_deviceAddress);  Serial.println(")");
  Serial.print("VCELL HIGH BYTE....: 0x"); printHex(vcell_h); Serial.print("\t("); Serial.print(vcell_h);  Serial.println(")");
  Serial.print("VCELL LOW BYTE.....: 0x"); printHex(vcell_l); Serial.print("\t("); Serial.print(vcell_l);  Serial.println(")");
  Serial.print("SOC HIGH BYTE......: 0x"); printHex(soc_h); Serial.print("\t("); Serial.print(soc_h);  Serial.println(")");
  Serial.print("SOC LOW BYTE.......: 0x"); printHex(soc_l); Serial.print("\t("); Serial.print(soc_l);  Serial.println(")");
  Serial.print("MODE HIGH BYTE.....: 0x"); printHex(mode_h); Serial.print("\t("); Serial.print(mode_h);  Serial.println(")");
  Serial.print("MODE LOW BYTE......: 0x"); printHex(mode_l); Serial.print("\t("); Serial.print(mode_l);  Serial.println(")");
  Serial.print("VERSION HIGH BYTE..: 0x"); printHex(ver_h); Serial.print("\t("); Serial.print(ver_h);  Serial.println(")");
  Serial.print("VERSION LOW BYTE...: 0x"); printHex(ver_l); Serial.print("\t("); Serial.print(ver_l);  Serial.println(")");
  Serial.print("CONFIG HIGH BYTE...: 0x"); printHex(config_h); Serial.print("\t("); Serial.print(config_h);  Serial.println(")");
  Serial.print("CONFIG LOW BYTE....: 0x"); printHex(config_l); Serial.print("\t("); Serial.print(config_l);  Serial.println(")");

  //
  // The ADC is 12-bit; the 4 LSB's are not used (shift
  // the bits to right by 4).
  //
  uint16_t adc = toUint16(vcell_h, vcell_l) >> 4;

  //
  // The SOC (charge percent) is the HIGH byte
  // + the LOW BYTE divided by 256 (the LOW byte
  // is the decimal portion).
  //
  float percent = soc_h + (soc_l / 256.0);

  //
  // The voltage units are 1.25mV for the MAX17043
  // and 2.50mV for the MAX17044.
  //
  float voltage1 = adc * 1.25;
  float voltage2 = adc * 2.50;

  //
  // Compute the alert threshold.
  //
  uint8_t alertThreshold = 32 - constrain(config_l & 0x1F, 0, 31);

  Serial.println();
  Serial.println("COMPUTED VALUES");
  Serial.println("----------------------------------------");
  Serial.print("PERCENT............: "); Serial.print(percent); Serial.println("%");
  Serial.print("ADC................: "); Serial.println(adc);
  Serial.print("VOLTAGE (MAX17043).: "); Serial.print(voltage1); Serial.println(" mV");
  Serial.print("VOLTAGE (MAX17044).: "); Serial.print(voltage2); Serial.println(" mV");
  Serial.print("SLEEPING...........: "); Serial.println(bitRead(config_l, 7) == 1 ? "Yes" : "No");
  Serial.print("ALERT..............: "); Serial.println(bitRead(config_l, 5) == 1 ? "Yes" : "No");
  Serial.print("ALERT THRESHOLD....: "); Serial.print(alertThreshold); Serial.println("%");
  Serial.print("COMPENSATION.......: 0x"); printHex(config_h);
}

void loop()
{
}

//
// Ensure a byte is sent to the
// serial device with 2 characters.
//
void printHex(uint8_t b)
{
  if (b < 16)
  {
    Serial.print("0");
  }

  Serial.print(b, HEX);
}

//
// Read a single byte from a register.
//
uint8_t readRegister8(uint8_t address, uint8_t registerId)
{
  writeRegisterId(address, registerId);
  Wire.requestFrom(address, (uint8_t)1);
  return Wire.read();
}

//
// Write the register ID to the bus leaving it
// open for further communications. This is used
// for reading a register value.
//
void writeRegisterId(uint8_t address, uint8_t registerId)
{
  Wire.beginTransmission(address);
  Wire.write(registerId);
  Wire.endTransmission(false);
}

//
// Write a 16 bit unsigned integer to a specified
// register.
//
void writeRegister16(uint8_t address, uint8_t registerId, uint16_t data)
{
  Wire.beginTransmission(address);
  Wire.write(registerId);
  Wire.write(toHighByte(data));
  Wire.write(toLowByte(data));
  Wire.endTransmission(true);
}

//
// Finds the first MAX1704X device on the bus.
//
uint8_t findFirstDevice(uint16_t expectedVersion)
{
  uint8_t returnValue = 0;

  //
  // Scan for a device on usable addresses.
  //
  for (uint8_t currentAddress = 0x08; currentAddress <= 0x77; currentAddress++)
  {
    //
    // Attempt a connection.
    //
    byte version = readRegister8(currentAddress, REGISTER_VERSION + 1);

    //
    // Match the retrieved version to the expected version.
    //
    if (version == expectedVersion)
    {
      returnValue = currentAddress;
      break;
    }
  }

  return returnValue;
}
