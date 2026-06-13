#include "port_handler.h"


DXLPortHandler::DXLPortHandler()
 : open_state_(false)
{}

/* DXLPortHandler */
bool DXLPortHandler::getOpenState()
{
  return open_state_;
}

void DXLPortHandler::setOpenState(bool state)
{
  open_state_ = state;
}


using namespace DYNAMIXEL;

/* SerialPortHandler */
SerialPortHandler::SerialPortHandler(HardwareSerial& port, const int dir_pin)
 : DXLPortHandler(), port_(port), dir_pin_(dir_pin), baud_(57600)
{}

void SerialPortHandler::begin()
{
  begin(baud_);
}

void SerialPortHandler::begin(unsigned long baud)
{
#if defined(ARDUINO_OpenCM904)
  if(port_ == Serial1 && getOpenState() == false){
    Serial1.setDxlMode(true);
  }
#elif defined(ARDUINO_OpenRB)
  if(port_ == Serial1 && getOpenState() == false){
    pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
    delay(500); // Wait for the FET to turn on.
  }
#elif defined(ARDUINO_OpenCR)
  if(port_ == Serial3 && getOpenState() == false){
    pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
    digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  }
  delay(500); // Wait for the DYNAMIXEL to power up normally.
#endif

  baud_ = baud;
  port_.begin(baud_);
  mbedTXdelayus = 24000000 / baud;
  
  if(dir_pin_ != -1){
    pinMode(dir_pin_, OUTPUT);
    digitalWrite(dir_pin_, LOW);
    while(digitalRead(dir_pin_) != LOW);
  }

  setOpenState(true);
}

void SerialPortHandler::end(void)
{
#if defined(ARDUINO_OpenCR)
  if(port_ == Serial3 && getOpenState() == true){
    digitalWrite(BDPIN_DXL_PWR_EN, LOW);
  }
#endif
  port_.end();
  setOpenState(false);
}

int SerialPortHandler::available(void)
{
  return port_.available();
}

int SerialPortHandler::read()
{
  return port_.read();
}

size_t SerialPortHandler::write(uint8_t c)
{
  size_t ret = 0;
  if(dir_pin_ != -1){
    digitalWrite(dir_pin_, HIGH);
    while(digitalRead(dir_pin_) != HIGH);
  }

  ret = port_.write(c);

  if(dir_pin_ != -1){
    port_.flush();
    digitalWrite(dir_pin_, LOW);
    while(digitalRead(dir_pin_) != LOW);
  }

  return ret;
}

size_t SerialPortHandler::write(uint8_t *buf, size_t len)
{
  size_t ret;
  if(dir_pin_ != -1){
    digitalWrite(dir_pin_, HIGH);
    while(digitalRead(dir_pin_) != HIGH);
  }

  ret = port_.write(buf, len);

  if(dir_pin_ != -1){
    port_.flush();
#if defined(ARDUINO_ARCH_MBED)
  delayMicroseconds(mbedTXdelayus);
#endif
    digitalWrite(dir_pin_, LOW);
    while(digitalRead(dir_pin_) != LOW);
  }

  return ret;      
}

unsigned long SerialPortHandler::getBaud() const
{
  return baud_;
}


/* USBSerialPortHandler */
USBSerialPortHandler::USBSerialPortHandler(USB_SERIAL_CLASS& port)
 : DXLPortHandler(), port_(port)
{}

void USBSerialPortHandler::begin()
{
  port_.begin(1000000);
  setOpenState(true);
}

void USBSerialPortHandler::end(void)
{
  port_.end();
  setOpenState(false);
}

int USBSerialPortHandler::available(void)
{
  return port_.available();
}

int USBSerialPortHandler::read()
{
  return port_.read();
}

size_t USBSerialPortHandler::write(uint8_t c)
{
  size_t ret = 0;

  ret = port_.write(c);

  return ret;
}

size_t USBSerialPortHandler::write(uint8_t *buf, size_t len)
{
  size_t ret;

  ret = port_.write(buf, len);

  return ret;      
}

