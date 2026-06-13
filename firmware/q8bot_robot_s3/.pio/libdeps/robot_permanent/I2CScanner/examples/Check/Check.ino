#include "I2CScanner.h"

I2CScanner scanner;

const uint8_t num_addresses = 4;
const byte addresses[num_addresses] = { 0x20, 0x21, 0x40, 0x41 };
bool results[num_addresses] = { false, false, false, false};


void setup() 
{
	Serial.begin(9600);
	while (!Serial) {};

	scanner.Init();
}

void loop() 
{
	for (uint8_t index = 0; index < num_addresses; index++)
	{
		results[index] = scanner.Check(addresses[index]);
	}
	
	for (uint8_t index = 0; index < num_addresses; index++)
	{
		if (results[index])
		{
			Serial.print("Found device ");
			Serial.print(index);
			Serial.print(" at address ");
			Serial.println(addresses[index], HEX);
		}
	}
	delay(5000);
}
