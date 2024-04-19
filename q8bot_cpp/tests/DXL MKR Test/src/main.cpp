#include <Arduino.h>
#include <Dynamixel2Arduino.h>

// define DXL serial ports
#define DEBUG_SERIAL Serial
#define DXL_SERIAL   Serial1

// DXL params
const uint8_t DXL_ID = 11;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_DIR_PIN = PIN_PA12;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  delay(1000);

  // Extended Position control mode
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID);

  // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 1500);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, 500);
}

void loop() {

  // Set Goal Position in DEGRE value
  dxl.setGoalPosition(DXL_ID, 270.0, UNIT_DEGREE);
  delay(2000);

  dxl.setGoalPosition(DXL_ID, 120.0, UNIT_DEGREE);
  delay(2000);

  dxl.setGoalPosition(DXL_ID, 45.0, UNIT_DEGREE);
  delay(2000);

  dxl.setGoalPosition(DXL_ID, 0.0, UNIT_DEGREE);
  delay(2000);
}