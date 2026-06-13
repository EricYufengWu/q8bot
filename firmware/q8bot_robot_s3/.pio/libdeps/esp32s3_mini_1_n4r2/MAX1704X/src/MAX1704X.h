/*
 * MAX1704X Arduino Library for MAX17043 and MAX17044 Fuel Gauge.
 *
 * Copyright © 2018-2022 Daniel Porrey. All Rights Reserved.
 * https://github.com/porrey/max1704x
 *
 * This file is part of the MAX1704X Arduino Library.
 * 
 * The MAX1704X Arduino Library is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * 
 * The MAX1704X Arduino Library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with the MAX1704X Arduino Library. If not, 
 * see http://www.gnu.org/licenses/.
 */
#ifndef MAX1704X_H
#define MAX1704X_H

#include <Arduino.h>
#include <Wire.h>

#define toLowByte(w)              ((uint8_t) ((w) & 0xff))
#define toHighByte(w)             ((uint8_t) ((w) >> 8))
#define toUint16(highB, lowB)     ((uint16_t)((uint8_t)highB << 8) + (uint8_t)lowB)
#define BV(bit)                   (1 <<(bit))
#define setBit(byte, bit)         (byte |= BV(bit))
#define clearBit(byte, bit)       (byte &= ~BV(bit))
#define toggleBit(byte, bit)      (byte ^= BV(bit))

#define I2C_DEFAULT_ADDRESS 0x36

#define REGISTER_VCELL      0x02
#define REGISTER_SOC        0x04
#define REGISTER_MODE       0x06
#define REGISTER_VERSION    0x08
#define REGISTER_CONFIG     0x0C
#define REGISTER_COMMAND    0xFE

#define RESET_COMMAND       0x5400
#define QUICKSTART_MODE     0x4000

#define DEFER_ADDRESS       (uint8_t)0

#define MAX17043_mV          1.25
#define MAX17044_mV          2.50
class MAX1704X
{
  public:
    MAX1704X(float);
    bool begin();
    bool begin(bool);
    bool begin(bool, uint8_t);
    bool begin(uint8_t);
    bool begin(TwoWire*);
    bool begin(TwoWire*, uint8_t);
    bool begin(TwoWire*, bool, uint8_t);
#if defined(ESP8266) || defined(ESP32)
    bool begin(int, int);
    bool begin(int, int, uint8_t);
#endif
    uint8_t address();
    void address(uint8_t);
    uint16_t adc();
    float voltage();
    float percent();
    float percent(bool);
    uint16_t version();
    uint8_t compensation();
    void compensation(uint8_t);
    bool sleep();
    bool isSleeping();
    bool wake();
    void reset();
    void quickstart();
    bool alertIsActive();
    void clearAlert();
    uint8_t threshold();
    void threshold(uint8_t);
    bool deviceFound();
    uint8_t findFirstDevice();
    uint8_t findFirstDevice(uint16_t expectedVersion);

    //
    // obsolete - will be removed in future release
    //
    uint8_t getThreshold();
    void setThreshold(uint8_t);
    
  protected:
    TwoWire *_wire;
    uint8_t _address;
    float _voltageIncrement;
    uint8_t thresholdToConfig(uint8_t);
    uint8_t configToThreshold(uint8_t);
    uint16_t readRegister16(uint8_t);
    void writeRegisterId(uint8_t);
    void writeRegister16(uint8_t, uint16_t);
};
#endif
