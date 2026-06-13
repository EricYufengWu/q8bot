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
#include "MAX1704X.h"

MAX1704X::MAX1704X(float voltageIncrement)
{
  this->_voltageIncrement = voltageIncrement;
}

bool MAX1704X::begin()
{
  this->_address = I2C_DEFAULT_ADDRESS;
  return this->begin(true, this->_address);
}

bool MAX1704X::begin(bool initializeWire)
{
  this->_address = I2C_DEFAULT_ADDRESS;
  return this->begin(initializeWire, this->_address);
}

bool MAX1704X::begin(bool initializeWire, uint8_t address)
{
  bool returnValue = false;

  this->_address = address;

  if (initializeWire)
  {
    Serial.println("Wire.begin()");
    this->_wire = &Wire;
    this->_wire->begin();

    if (address > 0)
    {
      returnValue = this->deviceFound();
    }
  }
  else
  {
    returnValue = true;
  }

  return returnValue;
}

bool MAX1704X::begin(uint8_t address)
{
  return this->begin(true, address);
}

bool MAX1704X::begin(TwoWire* wire)
{
  this->_wire = wire;
  return this->begin(false);
}

bool MAX1704X::begin(TwoWire* wire, uint8_t address)
{
  this->_wire = wire;
  return this->begin(false, address);
}

bool MAX1704X::begin(TwoWire* wire, bool initializeWire, uint8_t address)
{
  this->_wire = wire;
  return this->begin(initializeWire, address);
}

#if defined(ESP8266) || defined(ESP32)
bool MAX1704X::begin(int sda, int scl)
{
  this->_wire = &Wire;
  this->_wire->begin(sda, scl);
  return this->deviceFound();
}

bool MAX1704X::begin(int sda, int scl, uint8_t address)
{
  this->_address = address;
  this->_wire = &Wire;
  this->_wire->begin(sda, scl);
  return this->deviceFound();
}
#endif

uint8_t MAX1704X::address()
{
  return this->_address;
}

void MAX1704X::address(uint8_t address)
{
  this->_address = address;
}

uint16_t MAX1704X::adc()
{
  uint16_t returnValue = 0;

  //
  // 12-bit ADC; Shift the result since the 4 LSB's are not used.
  //
  uint16_t registerValue = this->readRegister16(REGISTER_VCELL);
  returnValue = registerValue >> 4;

  return returnValue;
}

float MAX1704X::voltage()
{
  //
  // The MAX1704X has a 12-bit ADC measuring 0 to 5 or 10 Volts in differing
  // increments.
  //
  return (float)(this->adc() * this->_voltageIncrement);
}

float MAX1704X::percent()
{
  //
  // Read the 16-bit register value
  //
  uint16_t registerValue = this->readRegister16(REGISTER_SOC);

  //
  // The high byte is the percentage.
  //
  float percentage = (float)toHighByte(registerValue);

  //
  // The low byte contains additional resolution of 1/256%.
  //
  float fraction = (float)(toLowByte(registerValue) / 256.f);

  return percentage + fraction;
}

float MAX1704X::percent(bool constrainPercent)
{
  float returnValue = this->percent();

  if (constrainPercent)
  {
    returnValue = constrain(returnValue, (float)0, (float)100);
  }

  return returnValue;
}

uint16_t MAX1704X::version()
{
  return this->readRegister16(REGISTER_VERSION);
}

uint8_t MAX1704X::compensation()
{
  uint16_t value = this->readRegister16(REGISTER_CONFIG);
  return toHighByte(value);
}

void MAX1704X::compensation(uint8_t data)
{
  uint16_t value = this->readRegister16(REGISTER_CONFIG);
  uint8_t lowByte = toLowByte(value);
  uint16_t newValue = toUint16(data, lowByte);
  this->writeRegister16(REGISTER_CONFIG, newValue);
}

bool MAX1704X::sleep()
{
  uint16_t value = this->readRegister16(REGISTER_CONFIG);
  setBit(value, 7);
  this->writeRegister16(REGISTER_CONFIG, value);
  return this->isSleeping();
}

bool MAX1704X::isSleeping()
{
  //
  // The sleep bit is bit 7 in the config byte.
  //
  uint16_t value = this->readRegister16(REGISTER_CONFIG);
  return (value & 0x80);
}

bool MAX1704X::wake()
{
  uint16_t value = this->readRegister16(REGISTER_CONFIG);
  clearBit(value, 7);
  this->writeRegister16(REGISTER_CONFIG, value);
  return !this->isSleeping();
}

void MAX1704X::reset()
{
  //
  // Write RESET_COMMAND to the command register.
  //
  this->writeRegister16(REGISTER_COMMAND, RESET_COMMAND);
}

void MAX1704X::quickstart()
{
  //
  // Write QUICKSTART_MODE to the mode register.
  //
  this->writeRegister16(REGISTER_MODE, QUICKSTART_MODE);
}

bool MAX1704X::alertIsActive()
{
  //
  // The alert bit is bit 5 in the config byte.
  //
  uint16_t value = this->readRegister16(REGISTER_CONFIG);
  return (value & 0x20);
}

void MAX1704X::clearAlert()
{
  //
  // Get the config register value. Bit 5
  // contains the alert value.
  //
  uint16_t value = this->readRegister16(REGISTER_CONFIG);
  clearBit(value, 5);
  this->writeRegister16(REGISTER_CONFIG, value);
}

uint8_t MAX1704X::threshold()
{
  //
  // Get the config register value. The last 5 bits
  // contains the threshold in 2's complement form.
  //
  uint16_t config = this->readRegister16(REGISTER_CONFIG);

  //
  // Use only the last 5 bits of the config.
  //
  return configToThreshold(config & 0x001F);
}

//
// obsolete - will be removed in future release
//
uint8_t MAX1704X::getThreshold()
{
  return this->threshold();
}

void MAX1704X::threshold(uint8_t threshold)
{
  //
  // Get the config register value. The last 5 bits
  // contains the threshold in 2's complement form.
  //
  uint16_t config = this->readRegister16(REGISTER_CONFIG);

  //
  // Set the last 5 bits to 0.
  //
  config &= 0xFFE0;

  //
  // The value of threshold can be between 1% and 32%.
  //
  uint8_t configThreshold = this->thresholdToConfig(threshold);

  //
  // Combine the threshold with the config.
  //
  config = config + configThreshold;

  //
  // Write the new register value.
  //
  this->writeRegister16(REGISTER_CONFIG, config);
}

//
// obsolete - will be removed in future release
//
void MAX1704X::setThreshold(uint8_t threshold)
{
  this->setThreshold(threshold);
}

uint8_t MAX1704X::thresholdToConfig(uint8_t threshold)
{
  return 31 - (constrain(threshold, 1, 32) - 1);
}

uint8_t MAX1704X::configToThreshold(uint8_t config)
{
  return (32 - constrain(config, 0, 31));
}

uint16_t MAX1704X::readRegister16(uint8_t registerId)
{
  this->writeRegisterId(registerId);
  this->_wire->requestFrom((uint8_t)this->_address, (uint8_t)2);

  uint8_t highByte = this->_wire->read();
  uint8_t lowByte = this->_wire->read();

  uint16_t data = toUint16(highByte, lowByte);
  return data;
}

void MAX1704X::writeRegisterId(uint8_t registerId)
{
  this->_wire->beginTransmission(this->_address);
  this->_wire->write(registerId);
  this->_wire->endTransmission(false);
}

void MAX1704X::writeRegister16(uint8_t registerId, uint16_t data)
{
  this->_wire->beginTransmission(this->_address);
  this->_wire->write(registerId);
  this->_wire->write(toHighByte(data));
  this->_wire->write(toLowByte(data));
  this->_wire->endTransmission(true);
}

bool MAX1704X::deviceFound()
{
  bool returnValue = false;

  this->_wire->beginTransmission(this->_address);
  byte error = this->_wire->endTransmission(true);

  if (error == 0)
  {
    returnValue = true;
  }

  return returnValue;
}

uint8_t MAX1704X::findFirstDevice()
{
  return this->findFirstDevice(0x03);
}

//
// Finds the first MAX1704X device on the bus.
//
uint8_t MAX1704X::findFirstDevice(uint16_t expectedVersion)
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
    this->_wire->beginTransmission(currentAddress);
    this->_wire->write(REGISTER_VERSION + 1);
    this->_wire->endTransmission(false);
    this->_wire->requestFrom(currentAddress, (uint8_t)1);
    byte version =  this->_wire->read();

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
