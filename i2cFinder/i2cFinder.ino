// #define USE_T3
#ifdef USE_T3
//#include <i2c_t3.h>
#else
#include <Wire.h>
#endif

#define LED 13

void setup()
{
	pinMode(33, OUTPUT);
#ifdef USE_T3
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
#else
	Wire.begin();
#endif
	
	Serial.begin(9600);
	Serial.println("\nI2C Scanner");
	pinMode(LED, OUTPUT);
}
void loop()
{
	
	byte error, address;
	int nDevices;
	digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmission to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknow error at address 0x");
			if (address<16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");

	delay(250);           // wait 5 seconds for next scan
	digitalWrite(LED, LOW);   // turn the LED on (HIGH is the voltage level)
	delay(250);
}
