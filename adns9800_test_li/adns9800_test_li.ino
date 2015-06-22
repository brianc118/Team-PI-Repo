#include <EEPROM.h>
#include <spi4teensy3.h>
#include <ADNS9800.h>
//adns9800_test_li.ino

ADNS9800 adns;

void setup() {
	adns.begin();
}

void loop() {
  	if(adns.moved_){
  		// Reset the interrupt flag
        moved_ = false;

        // Read the ball motion from the ADNS-9800
        int16_t xy[2];
        adns.adnsBurstMotion(xy);
        Serial.println(x);
  	}
}
