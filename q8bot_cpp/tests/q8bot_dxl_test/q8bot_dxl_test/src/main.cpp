#include <Arduino.h>
#include <HardwareSerial.h>
#include <Dynamixel2Arduino.h>

// define DXL serial ports
HardwareSerial MySerial0(0);

// DXL params
const uint8_t DXL_1_ID = 11;
const uint8_t DXL_2_ID = 12;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_DIR_PIN = 8;
bool pingValue;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

Dynamixel2Arduino dxl(MySerial0, DXL_DIR_PIN);

void setup() {
  Serial.begin(115200);

  while(!Serial){
    delay(100);
  }

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  pingValue = dxl.ping(DXL_1_ID);
  dxl.ping(DXL_2_ID);

  delay(1000);

  // Extended Position control mode
  dxl.torqueOff(DXL_1_ID);
  dxl.torqueOff(DXL_2_ID);
  dxl.setOperatingMode(DXL_1_ID, OP_EXTENDED_POSITION);
  dxl.setOperatingMode(DXL_2_ID, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_1_ID);
  dxl.torqueOn(DXL_2_ID);

  // for Time-based Extended Pos, Profile velocity is the move duration (ms).
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_1_ID, 2000);
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_2_ID, 2000);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_1_ID, 200);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_2_ID, 200);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Set Goal Position in DEGREE value
  dxl.setGoalPosition(DXL_1_ID, 0.0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_2_ID, 90.0, UNIT_DEGREE);
  Serial.println(pingValue);
  delay(4000);

  dxl.setGoalPosition(DXL_1_ID, 180.0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_2_ID, 270.0, UNIT_DEGREE);
  Serial.println(pingValue);
  delay(4000);

  dxl.setGoalPosition(DXL_1_ID, 360.0, UNIT_DEGREE);
  dxl.setGoalPosition(DXL_2_ID, 450.0, UNIT_DEGREE);
  Serial.println(pingValue);
  delay(4000);

  while(1){
    Serial.println(pingValue);
    delay(1000);
  }
}


