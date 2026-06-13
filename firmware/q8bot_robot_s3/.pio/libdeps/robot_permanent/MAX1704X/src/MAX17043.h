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
#ifndef MAX17043_H
#define MAX17043_H

#include "MAX1704X.h"

extern MAX1704X FuelGauge;

//
// The MAX17043 is a one-cell device with a
// a voltage measurement range of 0 to 5 V in 1.25 mV increments.
//
MAX1704X FuelGauge = MAX1704X(MAX17043_mV);

#endif
