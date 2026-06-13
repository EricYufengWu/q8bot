#include "I2CScanner.h"

I2CScanner scanner;

const byte address;

void debug(byte address)
{
	Serial.print("Found at 0x");
	Serial.println(address, HEX);
}

void setup() 
{
	Serial.begin(9600);
	while (!Serial) {};

	scanner.Init();
}

void loop() 
{
	scanner.Execute(debug);
	delay(5000);
}
