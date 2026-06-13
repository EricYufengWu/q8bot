// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Example Environment
//
// - DYNAMIXEL: X series (except XL-320)
//              ID = 11, 12, 13, 14, 15, Baudrate = 4000000bps, Protocol 2.0
// - Controller: OpenCR 1.0
// - Library: DYNAMIXEL2Arduino
// - Software: Blender 2.93.1
//             Arduino IDE
//
// Author: David Park

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


Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

// DYNAMIXEL Configuration
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
// OpenMANIPULATOR has ID of 11 ~ 15 by default
const uint8_t DYNAMIXEL_ID[] = {1, 2, 3, 4, 5};
const uint8_t NUMBER_OF_JOINT = sizeof(DYNAMIXEL_ID)/sizeof(uint8_t);


// Data struct for SyncRead
typedef struct sync_read_data {
  int32_t present_position;
} __attribute__((packed)) sync_read_data_t;

// Sync Read Present Position
const uint16_t PRESENT_POSITION_ADDRESS = 132;
const uint16_t LENGTH_TO_SYNC_READ = 4;

// Sync Read
sync_read_data_t sync_read_data[NUMBER_OF_JOINT];
DYNAMIXEL::InfoSyncReadInst_t sync_read_information;
DYNAMIXEL::XELInfoSyncRead_t sync_read_dynamixel_info[NUMBER_OF_JOINT];


// Data struct for SyncWrite
typedef struct sync_write_data {
  int32_t goal_position;
} __attribute__((packed)) sync_write_data_t;

// Sync Write Goal Position
const uint16_t GOAL_POSITION_ADDRESS = 116; 
const uint16_t LENGTH_TO_SYNC_WRITE = 4;

// Sync Write
sync_write_data_t sync_write_data[NUMBER_OF_JOINT];
DYNAMIXEL::InfoSyncWriteInst_t sync_write_information;
DYNAMIXEL::XELInfoSyncWrite_t sync_write_dynamixel_info[NUMBER_OF_JOINT];


const uint16_t MAX_PACKET_BUFFER_LENGTH = 128;
uint8_t packet_buffer[MAX_PACKET_BUFFER_LENGTH];

int loop_counts = 2;  // Number of times to play the motion
int frame_time = 0;
unsigned long current_time = 0;
unsigned long saved_time = 0;

// The number of information in each frame. (Time + Position of each Joint)
const int MAX_MOTION_DATA_COLUMN_INDEX = 1 + NUMBER_OF_JOINT;

// Paste all keyframes.
const int MOTION_DATA [][MAX_MOTION_DATA_COLUMN_INDEX]{
//Time[ms],ID1,ID2,ID3,ID4,Hand,
{100,2047,1535,2957,1649,2039},
{200,2047,1535,2957,1650,2039},
{300,2047,1534,2956,1652,2039},
{400,2047,1533,2956,1655,2039},
{500,2047,1531,2954,1660,2039},
{600,2047,1529,2953,1666,2039},
{700,2047,1526,2951,1674,2039},
{800,2047,1523,2949,1682,2039},
{900,2047,1520,2947,1692,2039},
{1000,2047,1516,2944,1702,2039},
{1100,2047,1511,2941,1714,2039},
{1200,2047,1507,2938,1726,2039},
{1300,2047,1502,2934,1740,2039},
{1400,2047,1497,2930,1754,2039},
{1500,2047,1492,2926,1769,2039},
{1600,2047,1486,2921,1784,2039},
{1700,2047,1480,2916,1800,2039},
{1800,2047,1474,2911,1817,2039},
{1900,2047,1468,2905,1834,2039},
{2000,2047,1462,2899,1851,2039},
{2100,2047,1455,2893,1869,2039},
{2200,2047,1448,2886,1887,2039},
{2300,2047,1442,2879,1906,2039},
{2400,2047,1435,2872,1924,2039},
{2500,2047,1428,2865,1943,2039},
{2600,2047,1421,2857,1962,2039},
{2700,2047,1415,2849,1980,2039},
{2800,2047,1408,2840,1999,2039},
{2900,2047,1401,2831,2018,2039},
{3000,2047,1394,2822,2036,2039},
{3100,2047,1388,2813,2054,2039},
{3200,2047,1381,2803,2072,2039},
{3300,2047,1375,2793,2090,2039},
{3400,2047,1369,2783,2107,2039},
{3500,2047,1363,2772,2123,2039},
{3600,2047,1357,2761,2139,2039},
{3700,2047,1351,2750,2155,2039},
{3800,2047,1346,2738,2170,2039},
{3900,2047,1341,2726,2184,2039},
{4000,2047,1336,2714,2197,2039},
{4100,2047,1331,2702,2209,2039},
{4200,2047,1327,2689,2221,2039},
{4300,2047,1323,2676,2232,2039},
{4400,2047,1320,2662,2241,2039},
{4500,2047,1317,2648,2250,2039},
{4600,2047,1314,2634,2257,2039},
{4700,2047,1312,2620,2263,2039},
{4800,2047,1310,2605,2268,2039},
{4900,2047,1309,2590,2272,2039},
{5000,2047,1308,2575,2274,2039},
{5100,2047,1308,2559,2274,2039},
{5200,2048,1308,2543,2274,2039},
{5300,2049,1308,2527,2274,2039},
{5400,2052,1308,2510,2274,2039},
{5500,2056,1308,2493,2274,2039},
{5600,2061,1308,2476,2274,2039},
{5700,2067,1308,2459,2274,2039},
{5800,2074,1308,2442,2274,2039},
{5900,2082,1308,2424,2274,2039},
{6000,2091,1308,2407,2274,2039},
{6100,2100,1308,2389,2274,2039},
{6200,2110,1308,2372,2274,2039},
{6300,2121,1308,2354,2274,2039},
{6400,2133,1308,2336,2274,2039},
{6500,2145,1308,2318,2274,2039},
{6600,2158,1308,2301,2274,2039},
{6700,2171,1308,2283,2274,2039},
{6800,2184,1308,2265,2274,2039},
{6900,2198,1308,2248,2274,2039},
{7000,2213,1308,2231,2274,2039},
{7100,2227,1308,2214,2274,2039},
{7200,2242,1308,2197,2274,2039},
{7300,2257,1308,2180,2274,2039},
{7400,2272,1308,2164,2274,2039},
{7500,2288,1308,2147,2274,2039},
{7600,2303,1308,2131,2274,2039},
{7700,2318,1308,2116,2274,2039},
{7800,2334,1308,2101,2274,2039},
{7900,2349,1308,2086,2274,2039},
{8000,2364,1308,2071,2274,2039},
{8100,2379,1308,2057,2274,2039},
{8200,2393,1308,2044,2274,2039},
{8300,2408,1308,2030,2274,2039},
{8400,2422,1308,2018,2274,2039},
{8500,2435,1308,2006,2274,2039},
{8600,2448,1308,1994,2274,2039},
{8700,2461,1308,1983,2274,2039},
{8800,2473,1308,1972,2274,2039},
{8900,2485,1308,1963,2274,2039},
{9000,2495,1308,1953,2274,2039},
{9100,2506,1308,1945,2274,2039},
{9200,2515,1308,1937,2274,2039},
{9300,2524,1308,1930,2274,2039},
{9400,2532,1308,1924,2274,2039},
{9500,2539,1308,1918,2274,2039},
{9600,2545,1308,1913,2274,2039},
{9700,2550,1308,1909,2274,2039},
{9800,2554,1308,1906,2274,2039},
{9900,2556,1308,1904,2274,2039},
{10000,2558,1308,1902,2274,2039},
{10100,2559,1308,1902,2274,2039},
{10200,2559,1308,1902,2274,2039},
{10300,2559,1309,1903,2274,2039},
{10400,2559,1312,1903,2274,2039},
{10500,2559,1315,1904,2273,2039},
{10600,2559,1319,1906,2273,2039},
{10700,2559,1323,1908,2272,2039},
{10800,2559,1329,1910,2271,2039},
{10900,2559,1335,1912,2271,2039},
{11000,2559,1342,1914,2270,2039},
{11100,2559,1349,1917,2269,2039},
{11200,2559,1357,1920,2267,2039},
{11300,2559,1365,1923,2266,2039},
{11400,2559,1374,1926,2265,2039},
{11500,2559,1384,1930,2264,2039},
{11600,2559,1394,1933,2262,2039},
{11700,2559,1404,1937,2261,2039},
{11800,2559,1414,1941,2259,2039},
{11900,2559,1425,1945,2258,2039},
{12000,2559,1436,1949,2256,2039},
{12100,2559,1448,1953,2254,2039},
{12200,2559,1459,1957,2253,2039},
{12300,2559,1471,1961,2251,2039},
{12400,2559,1483,1966,2249,2039},
{12500,2559,1495,1970,2248,2039},
{12600,2559,1507,1974,2246,2039},
{12700,2559,1519,1979,2244,2039},
{12800,2559,1531,1983,2243,2039},
{12900,2559,1542,1987,2241,2039},
{13000,2559,1554,1992,2239,2039},
{13100,2559,1566,1996,2238,2039},
{13200,2559,1577,2000,2236,2039},
{13300,2559,1588,2004,2234,2039},
{13400,2559,1599,2008,2233,2039},
{13500,2559,1610,2012,2231,2039},
{13600,2559,1620,2016,2230,2039},
{13700,2559,1630,2019,2229,2039},
{13800,2559,1639,2023,2227,2039},
{13900,2559,1648,2026,2226,2039},
{14000,2559,1656,2029,2225,2039},
{14100,2559,1664,2032,2224,2039},
{14200,2559,1672,2035,2222,2039},
{14300,2559,1678,2037,2222,2039},
{14400,2559,1685,2039,2221,2039},
{14500,2559,1690,2041,2220,2039},
{14600,2559,1695,2043,2219,2039},
{14700,2559,1699,2044,2219,2039},
{14800,2559,1702,2045,2218,2039},
{14900,2559,1704,2046,2218,2039},
{15000,2559,1705,2047,2218,2039},
{15100,2559,1706,2047,2218,2039},
{15200,2559,1706,2047,2218,2038},
{15300,2559,1706,2047,2218,2033},
{15400,2559,1706,2047,2218,2024},
{15500,2559,1706,2047,2218,2013},
{15600,2559,1706,2047,2218,1998},
{15700,2559,1706,2047,2218,1981},
{15800,2559,1706,2047,2218,1961},
{15900,2559,1706,2047,2218,1939},
{16000,2559,1706,2047,2218,1914},
{16100,2559,1706,2047,2218,1887},
{16200,2559,1706,2047,2218,1857},
{16300,2559,1706,2047,2218,1826},
{16400,2559,1706,2047,2218,1793},
{16500,2559,1706,2047,2218,1758},
{16600,2559,1706,2047,2218,1722},
{16700,2559,1706,2047,2218,1684},
{16800,2559,1706,2047,2218,1645},
{16900,2559,1706,2047,2218,1605},
{17000,2559,1706,2047,2218,1564},
{17100,2559,1706,2047,2218,1522},
{17200,2559,1706,2047,2218,1479},
{17300,2559,1706,2047,2218,1436},
{17400,2559,1706,2047,2218,1393},
{17500,2559,1706,2047,2218,1349},
{17600,2559,1706,2047,2218,1304},
{17700,2559,1706,2047,2218,1260},
{17800,2559,1706,2047,2218,1216},
{17900,2559,1706,2047,2218,1173},
{18000,2559,1706,2047,2218,1130},
{18100,2559,1706,2047,2218,1087},
{18200,2559,1706,2047,2218,1045},
{18300,2559,1706,2047,2218,1004},
{18400,2559,1706,2047,2218,964},
{18500,2559,1706,2047,2218,925},
{18600,2559,1706,2047,2218,887},
{18700,2559,1706,2047,2218,851},
{18800,2559,1706,2047,2218,816},
{18900,2559,1706,2047,2218,783},
{19000,2559,1706,2047,2218,752},
{19100,2559,1706,2047,2218,722},
{19200,2559,1706,2047,2218,695},
{19300,2559,1706,2047,2218,670},
{19400,2559,1706,2047,2218,648},
{19500,2559,1706,2047,2218,628},
{19600,2559,1706,2047,2218,611},
{19700,2559,1706,2047,2218,596},
{19800,2559,1706,2047,2218,585},
{19900,2559,1706,2047,2218,576},
{20000,2559,1706,2047,2218,571},
{20100,2559,1706,2047,2218,570},
{20200,2559,1705,2047,2218,570},
{20300,2559,1704,2047,2220,570},
{20400,2559,1702,2047,2224,570},
{20500,2559,1699,2047,2228,570},
{20600,2559,1695,2047,2234,570},
{20700,2559,1690,2047,2240,570},
{20800,2559,1685,2047,2248,570},
{20900,2559,1678,2047,2257,570},
{21000,2559,1672,2047,2266,570},
{21100,2559,1664,2047,2277,570},
{21200,2559,1656,2047,2288,570},
{21300,2559,1648,2047,2300,570},
{21400,2559,1639,2047,2313,570},
{21500,2559,1630,2047,2326,570},
{21600,2559,1620,2047,2340,570},
{21700,2559,1610,2047,2355,570},
{21800,2559,1599,2047,2370,570},
{21900,2559,1588,2047,2386,570},
{22000,2559,1577,2047,2402,570},
{22100,2559,1566,2047,2418,570},
{22200,2559,1554,2047,2434,570},
{22300,2559,1542,2047,2451,570},
{22400,2559,1531,2047,2468,570},
{22500,2559,1519,2047,2485,570},
{22600,2559,1507,2047,2502,570},
{22700,2559,1495,2047,2519,570},
{22800,2559,1483,2047,2536,570},
{22900,2559,1471,2047,2553,570},
{23000,2559,1459,2047,2570,570},
{23100,2559,1448,2047,2586,570},
{23200,2559,1436,2047,2602,570},
{23300,2559,1425,2047,2618,570},
{23400,2559,1414,2047,2634,570},
{23500,2559,1404,2047,2649,570},
{23600,2559,1394,2047,2664,570},
{23700,2559,1384,2047,2678,570},
{23800,2559,1374,2047,2691,570},
{23900,2559,1365,2047,2704,570},
{24000,2559,1357,2047,2716,570},
{24100,2559,1349,2047,2727,570},
{24200,2559,1342,2047,2738,570},
{24300,2559,1335,2047,2747,570},
{24400,2559,1329,2047,2756,570},
{24500,2559,1323,2047,2764,570},
{24600,2559,1319,2047,2770,570},
{24700,2559,1315,2047,2776,570},
{24800,2559,1312,2047,2780,570},
{24900,2559,1309,2047,2784,570},
{25000,2559,1308,2047,2786,570},
{25100,2559,1308,2047,2786,570},
{25200,2558,1308,2047,2786,570},
{25300,2554,1308,2047,2786,570},
{25400,2548,1308,2047,2786,570},
{25500,2540,1308,2047,2786,570},
{25600,2530,1308,2047,2786,570},
{25700,2518,1308,2047,2786,570},
{25800,2504,1308,2047,2786,570},
{25900,2489,1308,2047,2786,570},
{26000,2471,1308,2047,2786,570},
{26100,2452,1308,2047,2786,570},
{26200,2432,1308,2047,2786,570},
{26300,2410,1308,2047,2786,570},
{26400,2387,1308,2047,2786,570},
{26500,2363,1308,2047,2786,570},
{26600,2338,1308,2047,2786,570},
{26700,2311,1308,2047,2786,570},
{26800,2284,1308,2047,2786,570},
{26900,2256,1308,2047,2786,570},
{27000,2228,1308,2047,2786,570},
{27100,2199,1308,2047,2786,570},
{27200,2169,1308,2047,2786,570},
{27300,2139,1308,2047,2786,570},
{27400,2108,1308,2047,2786,570},
{27500,2078,1308,2047,2786,570},
{27600,2047,1308,2047,2786,570},
{27700,2016,1308,2047,2786,570},
{27800,1986,1308,2047,2786,570},
{27900,1955,1308,2047,2786,570},
{28000,1925,1308,2047,2786,570},
{28100,1895,1308,2047,2786,570},
{28200,1866,1308,2047,2786,570},
{28300,1838,1308,2047,2786,570},
{28400,1810,1308,2047,2786,570},
{28500,1783,1308,2047,2786,570},
{28600,1756,1308,2047,2786,570},
{28700,1731,1308,2047,2786,570},
{28800,1707,1308,2047,2786,570},
{28900,1684,1308,2047,2786,570},
{29000,1662,1308,2047,2786,570},
{29100,1642,1308,2047,2786,570},
{29200,1623,1308,2047,2786,570},
{29300,1605,1308,2047,2786,570},
{29400,1590,1308,2047,2786,570},
{29500,1576,1308,2047,2786,570},
{29600,1564,1308,2047,2786,570},
{29700,1554,1308,2047,2786,570},
{29800,1546,1308,2047,2786,570},
{29900,1540,1308,2047,2786,570},
{30000,1536,1308,2047,2786,570},
{30100,1535,1308,2047,2786,570},
{30200,1535,1308,2047,2786,570},
{30300,1535,1309,2047,2784,570},
{30400,1535,1312,2047,2782,570},
{30500,1535,1315,2047,2778,570},
{30600,1535,1319,2047,2774,570},
{30700,1535,1323,2047,2768,570},
{30800,1535,1329,2047,2762,570},
{30900,1535,1335,2047,2755,570},
{31000,1535,1342,2047,2747,570},
{31100,1535,1349,2047,2739,570},
{31200,1535,1357,2047,2730,570},
{31300,1535,1365,2047,2720,570},
{31400,1535,1374,2047,2710,570},
{31500,1535,1384,2047,2699,570},
{31600,1535,1394,2047,2688,570},
{31700,1535,1404,2047,2676,570},
{31800,1535,1414,2047,2664,570},
{31900,1535,1425,2047,2652,570},
{32000,1535,1436,2047,2639,570},
{32100,1535,1448,2047,2626,570},
{32200,1535,1459,2047,2613,570},
{32300,1535,1471,2047,2600,570},
{32400,1535,1483,2047,2586,570},
{32500,1535,1495,2047,2573,570},
{32600,1535,1507,2047,2559,570},
{32700,1535,1519,2047,2545,570},
{32800,1535,1531,2047,2532,570},
{32900,1535,1542,2047,2518,570},
{33000,1535,1554,2047,2505,570},
{33100,1535,1566,2047,2492,570},
{33200,1535,1577,2047,2479,570},
{33300,1535,1588,2047,2466,570},
{33400,1535,1599,2047,2453,570},
{33500,1535,1610,2047,2441,570},
{33600,1535,1620,2047,2430,570},
{33700,1535,1630,2047,2418,570},
{33800,1535,1639,2047,2408,570},
{33900,1535,1648,2047,2397,570},
{34000,1535,1656,2047,2388,570},
{34100,1535,1664,2047,2379,570},
{34200,1535,1672,2047,2370,570},
{34300,1535,1678,2047,2363,570},
{34400,1535,1685,2047,2356,570},
{34500,1535,1690,2047,2349,570},
{34600,1535,1695,2047,2344,570},
{34700,1535,1699,2047,2340,570},
{34800,1535,1702,2047,2336,570},
{34900,1535,1704,2047,2334,570},
{35000,1535,1705,2047,2332,570},
{35100,1535,1706,2047,2331,570},
{35200,1535,1706,2047,2331,571},
{35300,1535,1706,2047,2331,576},
{35400,1535,1706,2047,2331,585},
{35500,1535,1706,2047,2331,596},
{35600,1535,1706,2047,2331,611},
{35700,1535,1706,2047,2331,628},
{35800,1535,1706,2047,2331,648},
{35900,1535,1706,2047,2331,670},
{36000,1535,1706,2047,2331,695},
{36100,1535,1706,2047,2331,722},
{36200,1535,1706,2047,2331,752},
{36300,1535,1706,2047,2331,783},
{36400,1535,1706,2047,2331,816},
{36500,1535,1706,2047,2331,851},
{36600,1535,1706,2047,2331,887},
{36700,1535,1706,2047,2331,925},
{36800,1535,1706,2047,2331,964},
{36900,1535,1706,2047,2331,1004},
{37000,1535,1706,2047,2331,1045},
{37100,1535,1706,2047,2331,1087},
{37200,1535,1706,2047,2331,1130},
{37300,1535,1706,2047,2331,1173},
{37400,1535,1706,2047,2331,1216},
{37500,1535,1706,2047,2331,1260},
{37600,1535,1706,2047,2331,1305},
{37700,1535,1706,2047,2331,1349},
{37800,1535,1706,2047,2331,1393},
{37900,1535,1706,2047,2331,1436},
{38000,1535,1706,2047,2331,1479},
{38100,1535,1706,2047,2331,1522},
{38200,1535,1706,2047,2331,1564},
{38300,1535,1706,2047,2331,1605},
{38400,1535,1706,2047,2331,1645},
{38500,1535,1706,2047,2331,1684},
{38600,1535,1706,2047,2331,1722},
{38700,1535,1706,2047,2331,1758},
{38800,1535,1706,2047,2331,1793},
{38900,1535,1706,2047,2331,1826},
{39000,1535,1706,2047,2331,1857},
{39100,1535,1706,2047,2331,1887},
{39200,1535,1706,2047,2331,1914},
{39300,1535,1706,2047,2331,1939},
{39400,1535,1706,2047,2331,1961},
{39500,1535,1706,2047,2331,1981},
{39600,1535,1706,2047,2331,1998},
{39700,1535,1706,2047,2331,2013},
{39800,1535,1706,2047,2331,2024},
{39900,1535,1706,2047,2331,2033},
{40000,1535,1706,2047,2331,2038},
{40100,1535,1706,2047,2331,2040},
{40200,1536,1705,2047,2331,2040},
{40300,1537,1704,2048,2331,2040},
{40400,1540,1702,2050,2331,2040},
{40500,1544,1699,2052,2331,2040},
{40600,1549,1695,2055,2331,2039},
{40700,1555,1691,2058,2331,2039},
{40800,1561,1685,2062,2331,2039},
{40900,1569,1679,2066,2331,2039},
{41000,1577,1673,2071,2331,2039},
{41100,1586,1666,2077,2331,2039},
{41200,1596,1658,2083,2331,2039},
{41300,1607,1650,2089,2331,2039},
{41400,1618,1641,2096,2330,2039},
{41500,1630,1632,2104,2330,2039},
{41600,1642,1623,2111,2330,2039},
{41700,1655,1613,2120,2330,2039},
{41800,1668,1603,2128,2329,2039},
{41900,1681,1592,2137,2329,2039},
{42000,1695,1581,2147,2328,2039},
{42100,1710,1570,2157,2328,2039},
{42200,1724,1559,2167,2327,2039},
{42300,1739,1547,2177,2327,2039},
{42400,1754,1536,2188,2326,2039},
{42500,1769,1524,2199,2325,2039},
{42600,1784,1513,2210,2325,2039},
{42700,1799,1501,2222,2324,2039},
{42800,1814,1489,2234,2323,2039},
{42900,1829,1478,2246,2322,2039},
{43000,1843,1466,2259,2321,2039},
{43100,1858,1455,2271,2320,2039},
{43200,1873,1443,2284,2319,2039},
{43300,1887,1432,2297,2317,2039},
{43400,1901,1421,2310,2316,2039},
{43500,1914,1411,2323,2315,2039},
{43600,1927,1401,2337,2313,2039},
{43700,1940,1391,2350,2311,2039},
{43800,1952,1381,2364,2310,2039},
{43900,1964,1372,2378,2308,2039},
{44000,1975,1363,2392,2306,2039},
{44100,1986,1355,2406,2304,2039},
{44200,1996,1348,2420,2302,2039},
{44300,2005,1340,2434,2300,2039},
{44400,2013,1334,2448,2297,2039},
{44500,2021,1328,2462,2295,2039},
{44600,2027,1323,2476,2292,2039},
{44700,2033,1318,2490,2290,2039},
{44800,2038,1315,2504,2287,2039},
{44900,2042,1312,2518,2284,2039},
{45000,2045,1309,2531,2281,2039},
{45100,2046,1308,2545,2278,2039},
{45200,2047,1308,2559,2274,2039},
{45300,2047,1308,2572,2271,2039},
{45400,2047,1309,2586,2265,2039},
{45500,2047,1310,2599,2259,2039},
{45600,2047,1312,2612,2251,2039},
{45700,2047,1314,2625,2243,2039},
{45800,2047,1317,2638,2233,2039},
{45900,2047,1320,2651,2223,2039},
{46000,2047,1324,2663,2211,2039},
{46100,2047,1328,2676,2199,2039},
{46200,2047,1332,2688,2186,2039},
{46300,2047,1337,2700,2172,2039},
{46400,2047,1342,2712,2157,2039},
{46500,2047,1347,2723,2142,2039},
{46600,2047,1353,2735,2127,2039},
{46700,2047,1359,2746,2110,2039},
{46800,2047,1365,2757,2094,2039},
{46900,2047,1371,2768,2077,2039},
{47000,2047,1377,2779,2059,2039},
{47100,2047,1384,2789,2041,2039},
{47200,2047,1390,2799,2023,2039},
{47300,2047,1397,2809,2005,2039},
{47400,2047,1404,2818,1987,2039},
{47500,2047,1411,2828,1969,2039},
{47600,2047,1418,2837,1950,2039},
{47700,2047,1425,2846,1932,2039},
{47800,2047,1432,2854,1914,2039},
{47900,2047,1439,2862,1896,2039},
{48000,2047,1446,2870,1878,2039},
{48100,2047,1452,2878,1860,2039},
{48200,2047,1459,2885,1843,2039},
{48300,2047,1466,2892,1826,2039},
{48400,2047,1472,2899,1810,2039},
{48500,2047,1478,2905,1794,2039},
{48600,2047,1484,2911,1778,2039},
{48700,2047,1490,2917,1763,2039},
{48800,2047,1496,2922,1749,2039},
{48900,2047,1501,2927,1736,2039},
{49000,2047,1506,2932,1723,2039},
{49100,2047,1511,2936,1711,2039},
{49200,2047,1515,2940,1700,2039},
{49300,2047,1519,2943,1690,2039},
{49400,2047,1523,2947,1681,2039},
{49500,2047,1526,2949,1673,2039},
{49600,2047,1529,2952,1666,2039},
{49700,2047,1531,2954,1660,2039},
{49800,2047,1533,2955,1655,2039},
{49900,2047,1534,2956,1652,2039},
{50000,2047,1535,2957,1650,2039},
{50100,2047,1535,2957,1649,2039}
};

// The number of key frames.
const int TOTAL_FRAMES = sizeof(MOTION_DATA) / sizeof(MOTION_DATA[0]);

void setup()
{
  int time_between_frame = (MOTION_DATA[2][0] - MOTION_DATA[1][0]) * 2;

  // Open serial port for debugging
  DEBUG_SERIAL.begin(115200);
  while(!DEBUG_SERIAL);

  // Port Baudrate to communicate with connected DYNAMIXELs. If not matched, you will meet communication fail.
  dxl.begin(4000000);
  
  delay(100);  // Allow time for DYNAMIXEL to finish boot up

  // Recommended Protocol Version is "2.0" Note that DYNAMIXEL Protocol for each product may differ depending on your model in use.
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  InitDXL(time_between_frame);  // Initialize the DYNAMIXEL
  InitSyncRead();
  InitSyncWrite();
  delay(2000);
  InitPose(MOTION_DATA, time_between_frame);  // Pass a pose
}

void loop()
{ 
  // Pass the motion data
  PlayMotion(MOTION_DATA);
}

void InitDXL(int frame_time)
{
  uint8_t index = 0;
  dxl.torqueOff(DXL_BROADCAST_ID);

  // Ping DYNAMIXELs. If failed, check your Baudrate and pysical wiring connection.
  for (index = 0; index < NUMBER_OF_JOINT; index++)
  {
    if (!dxl.ping(DYNAMIXEL_ID[index])) {
      DEBUG_SERIAL.print("[ERROR] Failed to connect DYNAMIXEL ");
      DEBUG_SERIAL.println(DYNAMIXEL_ID[index]);
    } else {
      // Set the Drive Mode as Time-based mode.
      dxl.writeControlTableItem(DRIVE_MODE, DYNAMIXEL_ID[index], 4);
      // Lower Return Delay Time enhances the responsiveness.
      dxl.writeControlTableItem(RETURN_DELAY_TIME, DYNAMIXEL_ID[index], 0);
      dxl.setOperatingMode(DYNAMIXEL_ID[index], OP_POSITION);
      // Time-based should be larger than frame time.
      dxl.writeControlTableItem(PROFILE_VELOCITY, DYNAMIXEL_ID[index], frame_time);
    }
  }
  dxl.torqueOn(DXL_BROADCAST_ID);
  delay(100);
}

// Initialize SyncRead packet in the structure
void InitSyncRead()
{
  uint8_t index = 0;

  sync_read_information.packet.p_buf = packet_buffer;
  sync_read_information.packet.buf_capacity = MAX_PACKET_BUFFER_LENGTH;
  sync_read_information.packet.is_completed = false;
  sync_read_information.addr = PRESENT_POSITION_ADDRESS;
  sync_read_information.addr_length = LENGTH_TO_SYNC_READ;
  sync_read_information.p_xels = sync_read_dynamixel_info;
  sync_read_information.xel_count = 0;  

  for (index = 0; index < NUMBER_OF_JOINT ; index++) {
    sync_read_dynamixel_info[index].id = DYNAMIXEL_ID[index];
    sync_read_dynamixel_info[index].p_recv_buf = (uint8_t*)&sync_read_data[index];
    sync_read_information.xel_count++;
  }
  sync_read_information.is_info_changed = true;

  ReadPresentPosition();
}

// Initialize SyncWrite packet frame in the structure
void InitSyncWrite()
{
  uint8_t index = 0;
  
  sync_write_information.packet.p_buf = nullptr;
  sync_write_information.packet.is_completed = false;
  sync_write_information.addr = GOAL_POSITION_ADDRESS;
  sync_write_information.addr_length = LENGTH_TO_SYNC_WRITE;
  sync_write_information.p_xels = sync_write_dynamixel_info;
  sync_write_information.xel_count = 0;

  for (index = 0; index < NUMBER_OF_JOINT; index++) {
    sync_write_dynamixel_info[index].id = DYNAMIXEL_ID[index];
    sync_write_dynamixel_info[index].p_data = (uint8_t*)&sync_write_data[index].goal_position;
    sync_write_information.xel_count++;
  }
}

void InitPose(const int init_pose[][MAX_MOTION_DATA_COLUMN_INDEX], int frame_time)
{
  uint8_t index = 0;
  // Store margin between Goal and Present Position.
  // This determines the DYNAMIXEL where to rotate in either CW or CCW.
  int arr_position_margin[NUMBER_OF_JOINT] = {};
  int arr_dxl_ready_state[NUMBER_OF_JOINT] = {};
  int init_steps = 1;
  int dxl_ready_count = 0;
  int position_tolerance = 5;

  for (index = 0; index < NUMBER_OF_JOINT; index++){
    if (index == 0) {
      DEBUG_SERIAL.println("[Present Position] ");
    }

    arr_position_margin[index] = init_pose[0][index+1] - sync_read_data[index].present_position;
    DEBUG_SERIAL.print("[Present Position_Setup] ID");
    DEBUG_SERIAL.print(sync_read_information.p_xels[index].id);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.println(sync_read_data[index].present_position);
    DEBUG_SERIAL.print("[Goal Position - Present Position] ID");
    DEBUG_SERIAL.print(sync_read_information.p_xels[index].id);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.println(arr_position_margin[index]);
  }

  DEBUG_SERIAL.println("");

  while(true) {
    // Count the number of ready DYNAMIXEL.
    for (index = 0; index < NUMBER_OF_JOINT; index++) {
      if (arr_dxl_ready_state[index] == 1) {
        dxl_ready_count++;
      } else {
        dxl_ready_count = 0;
        break;
      }
    }

    if(dxl_ready_count == NUMBER_OF_JOINT) {
      delay(1000);

      for (index = 0; index < NUMBER_OF_JOINT; index++) {
        sync_write_data[index].goal_position = init_pose[0][index+1];
        dxl.ledOff(DYNAMIXEL_ID[index]);
      }

      sync_write_information.is_info_changed = true;
      dxl.syncWrite(&sync_write_information);
      frame_time +=1;
      DEBUG_SERIAL.println("=== [Play Motion] ===");
      break;
    }

    if ( ElapsedTime() >= (unsigned long)(frame_time - 10)) {
      for (index=0; index < NUMBER_OF_JOINT; index++) {
        if (index == 0) {
          DEBUG_SERIAL.println("[Moving to the init pose...]");
        }

        // If result in Goal Position - Present Position is positive number, movement direction is in CCW.
        if (arr_position_margin[index] >= position_tolerance) {
          DEBUG_SERIAL.print("DIR :CCW, ");
          DEBUG_SERIAL.print("ID: ");
          DEBUG_SERIAL.print(sync_read_information.p_xels[index].id);
          //set position every 20 ms
          arr_position_margin[index] = arr_position_margin[index] - init_steps ;
          DEBUG_SERIAL.print("\t Target Pose: ");
          DEBUG_SERIAL.print(init_pose[0][index+1]);
          DEBUG_SERIAL.print("\t Present Pose: ");
          DEBUG_SERIAL.println(init_pose[0][index+1] - arr_position_margin[index]);
        } else if (arr_position_margin[index] <= -position_tolerance ) {
          // If result in Goal Position - Present Position is negative number, movement direction is in CW.
          DEBUG_SERIAL.print("DIR : CW, ");
          DEBUG_SERIAL.print("ID: ");
          DEBUG_SERIAL.print(sync_read_information.p_xels[index].id);
          arr_position_margin[index] = arr_position_margin[index] + init_steps ; 
          DEBUG_SERIAL.print("\t Target Pose: ");
          DEBUG_SERIAL.print(init_pose[0][index+1]);
          DEBUG_SERIAL.print("\t Present Pose: ");
          DEBUG_SERIAL.println(init_pose[0][index+1] - arr_position_margin[index]);
        } else {
          arr_dxl_ready_state[index] = 1; 
          dxl.ledOn(DYNAMIXEL_ID[index]);
          DEBUG_SERIAL.print("ID: ");
          DEBUG_SERIAL.print(sync_read_information.p_xels[index].id);
          DEBUG_SERIAL.println(" is ready to go.");
        }

        sync_write_data[index].goal_position = init_pose[0][index+1] - arr_position_margin[index];
      }
    }

    if ( ElapsedTime() >= (unsigned long)(frame_time)) {
      sync_write_information.is_info_changed = true;
      dxl.syncWrite(&sync_write_information);
      DEBUG_SERIAL.println(" SyncWrite.");
      saved_time = current_time;
    }
  }
}

void PlayMotion(const int MOTION_DATA[][MAX_MOTION_DATA_COLUMN_INDEX])
{
  // Time difference between the saved_time and current time.
  if ( ElapsedTime() >= (unsigned long)(MOTION_DATA[frame_time + 1][0] - MOTION_DATA[frame_time][0]) ) {
    // DisplayTime();
    // ReadPresentPosition();

    uint8_t index = 0;

    DEBUG_SERIAL.print("[Goal Position] : ");
    for (index = 0; index < NUMBER_OF_JOINT; index++) {
      sync_write_data[index].goal_position = MOTION_DATA[frame_time + 1][index + 1];
      DEBUG_SERIAL.print(sync_write_data[index].goal_position);
      DEBUG_SERIAL.print("\t");
    }
    DEBUG_SERIAL.println("");

    sync_write_information.is_info_changed = true;
    
    if(!dxl.syncWrite(&sync_write_information)) {
      DEBUG_SERIAL.println("[ERROR] Failed to Sync Write.");
    }

    saved_time = current_time;
    frame_time += 1;

    // When reached at the end of the motion frame array
    if (frame_time >= TOTAL_FRAMES -1 ) {
      DEBUG_SERIAL.println("=== [Motion Play Completed] ===");
      DEBUG_SERIAL.print("Total Played Frames = ");
      DEBUG_SERIAL.println(frame_time + 1);
      frame_time = 1;
      loop_counts--;
    }
  }
}

void DisplayTime()
{
  // Display current time, saved_time time, and elapsed time in two trajectory points. If not needed, comment out. 
  DEBUG_SERIAL.println("=== [Print Time (ms)] ===");
  DEBUG_SERIAL.print("Current: ");
  DEBUG_SERIAL.print(current_time);
  DEBUG_SERIAL.print("\t Saved: ");
  DEBUG_SERIAL.print(saved_time);
  DEBUG_SERIAL.print("\t Elapsed:");
  DEBUG_SERIAL.print(current_time - saved_time);
  DEBUG_SERIAL.print("\t Motion Frame:");
  DEBUG_SERIAL.println(MOTION_DATA[frame_time+1][0]);
}

void ReadPresentPosition()
{
  uint8_t index = 0;

  if(dxl.syncRead(&sync_read_information) > 0) {
    DEBUG_SERIAL.print("[Present Position] : ");
    for (index = 0; index < NUMBER_OF_JOINT; index++) {
      DEBUG_SERIAL.print(sync_read_data[index].present_position);
      DEBUG_SERIAL.print("\t");
    }
    DEBUG_SERIAL.println("");
  } else {
    DEBUG_SERIAL.println("[ERROR] Sync Read Failed");
  }
}

unsigned long ElapsedTime()
{
  current_time = millis();
  return (current_time - saved_time);
}