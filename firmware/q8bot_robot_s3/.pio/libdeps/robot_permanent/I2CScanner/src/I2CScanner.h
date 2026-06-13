#ifndef _I2CSCANNER_h
#define _I2CSCANNER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Wire.h>

typedef void(*I2C_Callback)(byte);

class I2CScanner
{
 protected:
	void inline printHeader();
	void inline printAdressResult(byte error, byte address);
	void inline printResult();
	byte inline scan(byte address);

 public:
	uint8_t Devices_Count;
	uint8_t Low_Address = 1;
	uint8_t High_Address = 127;
	
	void Init();

	bool Scan();
	bool Scan(byte address);
	bool Scan(byte addreses[], uint8_t length);

	bool Check();
	bool Check(byte address);
	bool Check(byte addreses[], uint8_t length);

	void Execute(I2C_Callback callback);
	void Execute(byte address, I2C_Callback callback);
	void Execute(byte addreses[], uint8_t length, I2C_Callback callback);
};

#endif

