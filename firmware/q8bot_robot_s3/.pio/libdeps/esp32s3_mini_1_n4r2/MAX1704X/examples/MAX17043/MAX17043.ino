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
#include "MAX17043.h"

// ***
// *** Connections:
// *** 1) Conect SDA pin to A4 on Uno
// *** 2) Connect SCL pin to A5 on Uno
// *** 3) Connect the GND pin to ground on the Uno.
// ***
// *** For other devices lookup the correct i2C
// *** (SDA and SCL) pins.
// ***

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
  if (FuelGauge.begin())
  {
    //
    // Reset the device.
    //
    Serial.println("Resetting device...");
    FuelGauge.reset();
    delay(250);

    //
    // Issue a quickstart command and wait
    // for the device to be ready.
    //
    Serial.println("Initiating quickstart mode...");
    FuelGauge.quickstart();
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
    Serial.println("The MAX17043 device was NOT found.\n");
    while (true);
  }

  //
  // Other ways to initialize:
  //
  // begin(bool initializeWire)
  // begin(bool initializeWire, uint32_t address)
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
  Serial.print("Address:       0x"); Serial.println(FuelGauge.address(), HEX);
  Serial.print("Version:       "); Serial.println(FuelGauge.version());
  Serial.print("ADC:           "); Serial.println(FuelGauge.adc());
  Serial.print("Voltage:       "); Serial.print(FuelGauge.voltage()); Serial.println(" mV");
  Serial.print("Percent:       "); Serial.print(FuelGauge.percent()); Serial.println("%");
  Serial.print("Is Sleeping:   "); Serial.println(FuelGauge.isSleeping() ? "Yes" : "No");
  Serial.print("Alert:         "); Serial.println(FuelGauge.alertIsActive() ? "Yes" : "No");
  Serial.print("Threshold:     "); Serial.print(FuelGauge.getThreshold()); Serial.println("%");
  Serial.print("Compensation:  0x"); Serial.println(FuelGauge.compensation(), HEX);
  Serial.println();
}

void sleepMode()
{
  if (!FuelGauge.isSleeping())
  {
    FuelGauge.sleep();

    if (FuelGauge.isSleeping())
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
  if (FuelGauge.isSleeping())
  {
    FuelGauge.wake();

    if (!FuelGauge.isSleeping())
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
  FuelGauge.reset();
  Serial.println("Fuel Gauge has been reset/rebooted.");
}

void quickStart()
{
  FuelGauge.quickstart();
  Serial.println("Quick start has been initiated on the Fuel Gauge.");
}

void clearAlert()
{
  FuelGauge.clearAlert();
  Serial.println("The alert has been cleared on the Fuel Gauge.");
}

void incrementThreshold()
{
  uint8_t threshold = FuelGauge.threshold();
  FuelGauge.threshold(++threshold);
  Serial.print("The alert threshold has been incremented to "); Serial.print(FuelGauge.threshold()); Serial.println(",");
}

void decrementThreshold()
{
  uint8_t threshold = FuelGauge.threshold();
  FuelGauge.threshold(--threshold);
  Serial.print("The alert threshold has been decremented to "); Serial.print(FuelGauge.threshold()); Serial.println(",");
}
