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

// ***
// *** Connections:
// *** 1) Conect SDA pin to A4 on Uno
// *** 2) Connect SCL pin to A5 on Uno
// *** 3) Connect the GND pin to ground on the Uno.
// ***
// *** For other devices lookup the correct i2C
// *** (SDA and SCL) pins.
// ***

//
// Define the MAX17403 fuel gauge. Use MAX17044_mV
// for the MAX17044.
//
MAX1704X _fuelGauge = MAX1704X(MAX17043_mV);

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
  // Initialize the fuel gauge.
  //
  if (_fuelGauge.begin(true, 0x32))
  {
    Serial.println("The MAX1704X device was found.\n");

    //
    // Reset the device.
    //
    Serial.println("Resetting device...");
    _fuelGauge.reset();
    delay(250);

    //
    // Issue a quickstart command and wait
    // for the device to be ready.
    //
    Serial.println("Initiating quickstart mode...");
    _fuelGauge.quickstart();
    delay(125);

    //
    // Display an initial reading.
    //
    Serial.println("Reading device...");
    Serial.println();
    displayReading();
    Serial.println();
    displayMenu();
  }
  else
  {
    Serial.println("The MAX1704X device was NOT found.\n");
    while (true);
  }

  //
  // Other ways to initialize:
  //
  // begin()
  // begin(bool initializeWire)
  // begin(int sda, int scl)          [esp only]
  // begin(int sda, int scl, uint8_t) [esp only]
  // bool begin(TwoWire* wire); [pass &Wire or &Wire1 for example]
  // bool begin(TwoWire* wire, uint8_t address);
  // bool begin(TwoWire* wire, bool initializeWire, uint8_t address);
}

void loop()
{
  //
  // Wait for serial input.
  //
  while (Serial.available() == 0)
  {
    delay(25);
  }

  char c = Serial.read();

  switch (c)
  {
    case 'M':
    case 'm':
      displayMenu();
      break;
    case 'D':
    case 'd':
      displayReading();
      break;
    case 'S':
    case 's':
      sleepMode();
      break;
    case 'W':
    case 'w':
      wakeMode();
      break;
    case 'Q':
    case 'q':
      quickStart();
      delay(500);
      displayReading();
      break;
    case 'C':
    case 'c':
      clearAlert();
      break;
    case 'R':
    case 'r':
      reset();
      break;
    case '+':
      incrementThreshold();
      break;
    case '-':
      decrementThreshold();
      break;
  }
}

void displayMenu()
{
  Serial.println("Enter an option in the serial input (M for menu):");
  Serial.println("D => Display a reading.");
  Serial.println("S => Enter sleep mode.");
  Serial.println("W => Wake.");
  Serial.println("Q => Quick start.");
  Serial.println("C => Clear alert.");
  Serial.println("R => Reset.");
  Serial.println("+ => Increment threshold.");
  Serial.println("- => Decrement threshold.");
  Serial.println();
}

void displayReading()
{
  //
  // Get the voltage, battery percent
  // and other properties.
  //
  Serial.println("Device Reading:");
  Serial.print("Address:       0x"); Serial.println(_fuelGauge.address(), HEX);
  Serial.print("Version:       "); Serial.println(_fuelGauge.version());
  Serial.print("ADC:           "); Serial.println(_fuelGauge.adc());
  Serial.print("Voltage:       "); Serial.print(_fuelGauge.voltage()); Serial.println(" mV");
  Serial.print("Percent:       "); Serial.print(_fuelGauge.percent()); Serial.println("%");
  Serial.print("Is Sleeping:   "); Serial.println(_fuelGauge.isSleeping() ? "Yes" : "No");
  Serial.print("Alert:         "); Serial.println(_fuelGauge.alertIsActive() ? "Yes" : "No");
  Serial.print("Threshold:     "); Serial.print(_fuelGauge.getThreshold()); Serial.println("%");
  Serial.print("Compensation:  0x"); Serial.println(_fuelGauge.compensation(), HEX);
  Serial.println();
}

void sleepMode()
{
  if (!_fuelGauge.isSleeping())
  {
    _fuelGauge.sleep();

    if (_fuelGauge.isSleeping())
    {
      Serial.println("Fuel Gauge put in sleep mode.");
    }
    else
    {
      Serial.println("Fuel Gauge failed to be put in sleep mode.");
    }
  }
  else
  {
    Serial.println("Fuel Gauge is already in sleep mode.");
  }
}

void wakeMode()
{
  if (_fuelGauge.isSleeping())
  {
    _fuelGauge.wake();

    if (!_fuelGauge.isSleeping())
    {
      Serial.println("Fuel Gauge is now awake.");
    }
    else
    {
      Serial.println("Failed to wake Fuel Gauge.");
    }
  }
  else
  {
    Serial.println("Fuel Gauge is already awake.");
  }
}

void reset()
{
  _fuelGauge.reset();
  Serial.println("Fuel Gauge has been reset/rebooted.");
}

void quickStart()
{
  _fuelGauge.quickstart();
  Serial.println("Quick start has been initiated on the Fuel Gauge.");
}

void clearAlert()
{
  _fuelGauge.clearAlert();
  Serial.println("The alert has been cleared on the Fuel Gauge.");
}

void incrementThreshold()
{
  uint8_t threshold = _fuelGauge.threshold();
  _fuelGauge.threshold(++threshold);
  Serial.print("The alert threshold has been incremented to "); Serial.print(_fuelGauge.threshold()); Serial.println(",");
}

void decrementThreshold()
{
  uint8_t threshold = _fuelGauge.threshold();
  _fuelGauge.threshold(--threshold);
  Serial.print("The alert threshold has been decremented to "); Serial.print(_fuelGauge.threshold()); Serial.println(",");
}
