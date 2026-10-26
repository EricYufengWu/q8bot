/*
Written by yufeng.wu0902@gmail.com

Initial setup helper code for q8bot. Configur brand new Dynamixel motors
for robot use: motor ID, baudrate, direction, control mode, offset...
Workflow:
Start with nothing connected and open the serial monitor
- Connect first new Dynamixel to the slot for ID 11 (front left vertical)
- Serial monitor should prompt confuguration successful
- Connect second new Dynamixel to the slot for ID 12 (front left horizontal)
- Serial monitor should prompt configuration successful
- Repeat until all 8 leg motors are configured and have baudrate set at 1M
- Connect the head Dynamixel last, for ID 19. It is daisy chained from the
  nearest leg servo: the head sits on the front centreline, roughly 35 mm from
  either front-outer servo, so run its cable to the free connector on the front
  outer motor (ID 11 or ID 13, whichever side is convenient). Both are the same
  distance; the Dynamixel bus does not care about the order on the chain.
- Program will now switch hardware serial to 1Mb and command all joints to
go to the starting condition and prompt you to install the legs. The head is
commanded to its centre and held there.

The head is not a ninth commanded joint: the ESP-NOW packet still carries the
eight leg angles, and the robot firmware broadcasts torque to ID 254, so ID 19
holds the centre position set here without any protocol change.
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Dynamixel2Arduino.h>

// Objects and Constants
HardwareSerial ser(0);
const uint32_t STARTING_BAUD = 57600;
const uint32_t FINAL_BAUD = 1000000;
const float DXL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_DIR_PIN = 8;
const uint8_t BROADCAST_ID = 254;
const uint8_t STARTING_ID = 1;

using namespace ControlTableItem;
Dynamixel2Arduino dxl(ser, 8);

// IDs 11-18 are the four leg pairs; ID 19 is the head, daisy chained from the
// nearest front-outer leg servo. The head takes the same +2048 homing offset as
// the other "first of pair" motors, so its mechanical centre reads as 0 deg
// (4096 ticks) in the same convention the robot firmware uses.
const uint8_t idList[9] = {11, 12, 13, 14, 15, 16, 17, 18, 19};
const uint8_t driveMode[9] = {4, 4, 5, 5, 4, 4, 5, 5, 4};
const uint8_t operateMode = 4;
const uint32_t homingOffset[9] = {2048, 4096, 4294965248, 4294963200, 2048, 4096, 4294965248, 4294963200, 2048};
const uint8_t baudRate = 3; // Corresponds to 1Mb baudrate
const int32_t idlePos[9]   = {4618, 5622, 4618, 5622, 4618, 5622, 4618, 5622, 4096};
const uint16_t moveTime = 1000;
uint8_t count = 0;
bool pingValue;

// put function declarations here:
void finishSetup();

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Begin Q8bot motor config");

  // Begin DXL comm at default baudrate
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}

void loop() {
  // Repeat param config for all 9 Dynamixel motors here (8 leg + 1 head).
  if (count < sizeof(idList)) {
    // First check whether a motor with desired ID already exists. If so move to the next ID.
    dxl.begin(FINAL_BAUD);
    if (dxl.ping(idList[count])) {
      Serial.printf("Joint %d already set up.\n", idList[count]);
      dxl.writeControlTableItem(DRIVE_MODE, idList[count], driveMode[count]);
      delay(100);
      dxl.writeControlTableItem(OPERATING_MODE, idList[count], operateMode);
      delay(100);
      dxl.writeControlTableItem(HOMING_OFFSET, idList[count], homingOffset[count]);
      delay(100);
      count++;
    // If not, wait for the first new motor to connect and set it up for the ID
    } else {
      dxl.begin(STARTING_BAUD);
      if (dxl.ping(STARTING_ID)) {
        Serial.println("New motor connected!");
        dxl.writeControlTableItem(DRIVE_MODE, STARTING_ID, driveMode[count]);
        dxl.writeControlTableItem(OPERATING_MODE, STARTING_ID, operateMode);
        dxl.writeControlTableItem(HOMING_OFFSET, STARTING_ID, homingOffset[count]);
        // Finally set 1Mb baudrate and move to next ID
        Serial.print("DXL Params set. Changing ID to "); Serial.println(idList[count]);
        dxl.writeControlTableItem(ID, STARTING_ID, idList[count]);
        delay(100);
        dxl.writeControlTableItem(BAUD_RATE, idList[count], 3);
        delay(100);
        count++;
      } else {
        Serial.println("No new Dynamixel motors found.");
      }
      // Delay and wait for the next check
      delay(5000);
    }
  // When all motors have been configured, exit main loop and perform a final check.
  } else {
    Serial.println("Leave all motors connected. Switching to new baudrate.");
    delay(5000);
    finishSetup();
  }
}

void finishSetup() {
  // Reopen dxl port at 1M baudrate
  dxl.begin(FINAL_BAUD);
  dxl.torqueOn(BROADCAST_ID);
  for (int i = 0; i < sizeof(idList); i++) {
    Serial.printf("Motor ID %d connection ", idList[i]);
    if (dxl.ping(idList[i])) {
      Serial.println("successful.");
      dxl.writeControlTableItem(PROFILE_VELOCITY, idList[i], moveTime);
      dxl.writeControlTableItem(PROFILE_ACCELERATION, idList[i], moveTime / 3);
      dxl.writeControlTableItem(GOAL_POSITION, idList[i], idlePos[i]);
    } else {
      Serial.println("not found.");
    }
  }

  // Motor setup complete
  Serial.println("Motor setup complete. ID 19 (head) is centred and held.");
  Serial.println("You may continue to install legs on your Q8bot.");
  Serial.println("When finished, switch the robot off to disable torque.");
  while (1) {
    delay(100);
  }
}