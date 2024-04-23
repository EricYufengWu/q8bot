/*
  q8Dynamixel.h - Wrapper for the Robotis Dynamixel2Arduino library.
  Created by Eric Wu, April 21st, 2024.
  Released into the public domain.
*/
#ifndef q8Dynamixel_h
#define q8Dynamixel_h

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

class q8Dynamixel
{
  public:
    q8Dynamixel(Dynamixel2Arduino& dxl);
    void begin();
    bool checkComms(uint8_t ID);
    void enableTorque();
    void disableTorque();
    void setOpMode();
    void moveAll(float deg);

  private:
    Dynamixel2Arduino& _dxl; // Member variable to store the object of Dynamixel2Arduino
    uint32_t _baudrate = 1000000;
    float _protocolVersion = 2.0;
    static const uint8_t _idCount = 8;
    const uint8_t _DXL[_idCount] = {11, 12, 13, 14, 15, 16, 17, 18};
    const uint8_t _directionPin = 8;
    int32_t _deg2Dxl(float deg);
    float _dxl2Deg(int32_t dxlRaw);
};

#endif