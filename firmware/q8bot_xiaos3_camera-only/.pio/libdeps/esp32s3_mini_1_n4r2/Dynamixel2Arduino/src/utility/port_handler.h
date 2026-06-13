/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef DYNAMIXEL_PORT_HANDLER_HPP_
#define DYNAMIXEL_PORT_HANDLER_HPP_


#include <Arduino.h>


class DXLPortHandler
{
  public:
    DXLPortHandler();
    
    virtual void begin() = 0;
    virtual void end() = 0;
    virtual int available(void) = 0;
    virtual int read() = 0;
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(uint8_t *buf, size_t len) = 0;
    bool getOpenState();
    void setOpenState(bool);

  private:
    bool open_state_;
};


namespace DYNAMIXEL{

class SerialPortHandler : public DXLPortHandler
{
  public:
    SerialPortHandler(HardwareSerial& port, const int dir_pin = -1);

    virtual void begin() override;
    virtual void end() override;
    virtual int available(void) override;
    virtual int read() override;
    virtual size_t write(uint8_t) override;
    virtual size_t write(uint8_t *buf, size_t len) override;

    virtual void begin(unsigned long baud);
    virtual unsigned long getBaud() const;

  private:
    HardwareSerial& port_;
    const int dir_pin_;
    unsigned long baud_;
    unsigned int mbedTXdelayus;
};


#if defined(__OPENCR__)
  #define USB_SERIAL_CLASS USBSerial
#elif defined(__MK20DX128__) || defined(__MK20DX256__)
  #include <usb_serial.h>  // Teensy 3.0 and 3.1
  #define USB_SERIAL_CLASS usb_serial_class
#elif defined(_SAM3XA_)
  #include <UARTClass.h>  // Arduino Due
  #define USB_SERIAL_CLASS UARTClass
#elif defined(ARDUINO_AVR_LEONARDO)
  // Arduino Leonardo USB Serial Port
  #define USB_SERIAL_CLASS Serial_
#elif defined(ARDUINO_CommXEL)
  #include "USBSerial.h"
  #define USB_SERIAL_CLASS USBSerial
#else
  #define USB_SERIAL_CLASS HardwareSerial
#endif

class USBSerialPortHandler : public DXLPortHandler
{
  public:
    USBSerialPortHandler(USB_SERIAL_CLASS& port);

    virtual void begin() override;
    virtual void end() override;
    virtual int available(void) override;
    virtual int read() override;
    virtual size_t write(uint8_t) override;
    virtual size_t write(uint8_t *buf, size_t len) override;

  private:
    USB_SERIAL_CLASS& port_;
};

}//namespace DYNAMIXEL

#endif /* DYNAMIXEL_PORT_HANDLER_HPP_ */