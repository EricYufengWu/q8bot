# Librería Arduino I2CScanner
La librería I2CScanner implementa un escaner para localizar dispositivos I2C, y determinar si un dispositivo está conectado.

Más información https://www.luisllamas.es/libreria-arduino-i2cscanner/

## Instrucciones de uso
La librería I2CScanner se instancia a través de su constructor. El objeto contiene como variables el rango de direcciones para el escaneo 'Low_Address y 'High_Address. El objeto se inicializa a traves del método 'Init().

Se dispone de tres familias de métodos. Los métodos 'Scan funcionan de forma al Sketch tradicional I2CScanner, mostrando los resultados por la pantalla. Los métodos 'Check comprueban la existencia de los dispositivos, pero no muestran ninguna salida. Finalmente, las funciones Execute reciben una función de Callback como parámetro, y la ejecutan únicamente si el dispositivo está conectado.

Las tres familias de funciones tienen sobrecargas. Si no reciben ningún parámetro, actúan sobre el rango definido por 'Low_Address y 'High_Address. Si reciben una dirección, actúan sobre la misma. Si reciben un vector de direcciones actúan sobre las mismas.

### Constructor
```c++
I2CScanner();
```

### Uso de I2CScanner
```c++
	// Dispositivos encontrados en el último scanner
	uint8_t Devices_Count;
	
	// Rango para el scanner sin parámetros
	uint8_t Low_Address = 1;
	uint8_t High_Address = 127;
	
	// Inicialización
	void Init();

	// Funciones scanner, muestran resultado por serial
	bool Scan();
	bool Scan(byte address);
	bool Scan(byte addreses[], uint8_t length);

	// Funciones check, no muestran resultado por serial
	bool Check();
	bool Check(byte address);
	bool Check(byte addreses[], uint8_t length);

	// Funciones execute, ejecutan la funcion callback si el dispositivo existe
	void Execute(I2C_Callback callback);
	void Execute(byte address, I2C_Callback callback);
	void Execute(byte addreses[], uint8_t length, I2C_Callback callback);
```

## Ejemplos
La librería I2CScanner incluye los siguientes ejemplos para ilustrar su uso.

* Scanner: Ejemplo que muestra el uso de I2CScanner mostrando resultados por Serial.
```c++
 #include "I2CScanner.h"

I2CScanner scanner;

void setup() 
{
	Serial.begin(9600);
	while (!Serial) {};

	scanner.Init();
}

void loop() 
{
	scanner.Scan();
	delay(5000);
}
```

* Check: Ejemplo que muestra el uso de I2CScanner, almacenando los resultados de check en un array, que luego usariamos en el código.
```c++
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
```

* Execute: Ejemplo que muestra el uso de I2CScanner ejecutando funciones callback.
```c++
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
```
