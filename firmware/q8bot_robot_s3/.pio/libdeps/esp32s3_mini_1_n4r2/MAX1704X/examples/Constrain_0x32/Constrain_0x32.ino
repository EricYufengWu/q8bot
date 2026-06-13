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
  }
  else
  {
    Serial.println("The MAX1704X device was NOT found.\n");
    while (true);
  }
}

void loop()
{
  //
  // percent(true) constrains the percentage between 0 and 100%.
  //
  Serial.print("Battery Percent is "); Serial.print(_fuelGauge.percent(true)); Serial.println("%");
  delay(2000);
}
