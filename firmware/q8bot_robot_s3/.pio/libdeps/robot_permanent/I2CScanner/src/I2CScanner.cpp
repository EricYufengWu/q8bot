#include "I2CScanner.h"

void I2CScanner::printHeader()
{
	Serial.println(F("--- Scan started ---"));
}

void I2CScanner::printAdressResult(byte error, byte address)
{
	if (error == 0)
	{
		Serial.print(F("I2C device found at address 0x"));
		if (address < 16) Serial.print("0");
		Serial.print(address, HEX);
		Serial.println(F("  !"));

		Devices_Count++;
	}
	else if (error == 4)
	{
		Serial.print(F("Unknow error at address 0x"));
		if (address < 16) Serial.print("0");
		Serial.println(address, HEX);
	}
}

void I2CScanner::printResult()
{
	if (Devices_Count == 0) Serial.println(F("No I2C devices found"));
	Serial.println(F("--- Scan finished ---\n"));
}


byte I2CScanner::scan(byte address)
{
	Wire.beginTransmission(address);
	byte error = Wire.endTransmission();
	return error;
}

void I2CScanner::Init()
{
	Wire.begin();
}

bool I2CScanner::Scan()
{
	printHeader();
	
	Devices_Count = 0;
	for (byte address = Low_Address; address < High_Address; address++)
	{
		byte error = scan(address);
		printAdressResult(error, address);
	}

	printResult();
	return Devices_Count > 0;
}

bool I2CScanner::Scan(byte address)
{
	printHeader();

	Devices_Count = 0;
	byte error = scan(address);
	printAdressResult(error, address);

	printResult();
	return Devices_Count > 0;
}

bool I2CScanner::Scan(byte addreses[], uint8_t length)
{
	printHeader();

	Devices_Count = 0;
	for (byte index = 0; index < length; index++)
	{
		byte address = addreses[index];
		byte error = scan(address);
		printAdressResult(error, address);
	}

	printResult();
	return Devices_Count > 0;
}

bool I2CScanner::Check()
{
	Devices_Count = 0;
	for (byte address = Low_Address; address < High_Address; address++)
		if(scan(address) == 0) Devices_Count++;
	
	return Devices_Count > 0;
}

bool I2CScanner::Check(byte address)
{
	Devices_Count = 0;
	if (scan(address) == 0) Devices_Count++;
		
	return Devices_Count > 0;
}

bool I2CScanner::Check(byte addreses[], uint8_t length)
{
	Devices_Count = 0;
	for (byte index = 0; index < length; index++)
	{
		byte address = addreses[index];
		if (scan(address) == 0) Devices_Count++;
	}

	return Devices_Count > 0;
}

void I2CScanner::Execute(I2C_Callback callback)
{
	for (byte address = Low_Address; address < High_Address; address++)
	{
		byte error = scan(address);
		if (error == 0 && callback != nullptr) callback(address);
	}
}

void I2CScanner::Execute(byte address, I2C_Callback callback)
{
	byte error = scan(address);
	if (error == 0 && callback != nullptr) callback(address);
}

void I2CScanner::Execute(byte addreses[], uint8_t length, I2C_Callback callback)
{
	for (byte index = 0; index < length; index++)
	{
		byte address = addreses[index];
		byte error = scan(address);
		if (error == 0 && callback != nullptr) callback(address);
	}
}