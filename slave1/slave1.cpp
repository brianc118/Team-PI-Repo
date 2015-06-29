/*
 * slave1.cpp
 * Team Pi Slave 1 Code
 * 
 * The Slave 1 functions as the orientation/location processor by acquiring
 * data from the iNEMO LSM9DS0 IMU, and 16 light sensors. This data
 * is interpreted and returned to the master Teensy using Serial/UART
 * 
 * Dependencies:
 *  	SFE_LSM9DS0
 *  	EEPROMAnything
 * All other non core libraries are written by me.
 *
 * By Brian Chen
 * Originally authored in November 2014
 * Copyright (c) 2015 Team PI
 */


#include <WProgram.h>

#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <SPI.h>
#include <SFE_LSM9DS0.h>
#include <piCommon.h>
#include <slave1.h>
#include <DebugUtils.h>

#define LED 13

union float2bytes { float f; uint8_t b[sizeof(float)]; };
float2bytes f2b;


elapsedMillis ledElapsedTime;
bool ledState = true;
uint32_t ledBlinkTime = 500;
elapsedMillis SerialRequestTime;


float bearing = 0;
float bearing_offset = 0;

uint8_t outBuffer[22] = {0};

inline void commandRequestStandardPacket();


void calibIMUOffset(){
	bearing_offset = 0;
	for (int i = 0; i < 1000; i++){
		slave1.imu.read();
		slave1.imu.complementaryFilterBearing(0.98);
	}
	for (int i = 0; i < 50; i++){
		slave1.imu.read();
		slave1.imu.complementaryFilterBearing(0.98);
		bearing = -slave1.imu.yaw;

		bearing_offset += bearing;
		delay(1);
	}
	bearing_offset /= 50;
}

void calibLight(){
	Serial.println("place on white");
	while(!Serial.available()){};
	CLEARSERIAL();

	slave1.lightArray.calibWhite();
	PRINTARRAY(slave1.lightArray.white);
	Serial.println("place on green");

	while(!Serial.available()){};
	CLEARSERIAL();

	slave1.lightArray.calibGreen();
	slave1.lightArray.endCalib();
	PRINTARRAY(slave1.lightArray.green);
	Serial.println();
	Serial.println("calibrated references values");
	PRINTARRAY(slave1.lightArray.refData);
	Serial.print("bad light sensors: ");
	for (int i = 0; i < 16; i++){
		if (slave1.lightArray.refData[i] == 255){
			Serial.print(i + 1);
			Serial.print('\t');
		}
	}
	Serial.println();
	Serial.println("Send anything to continue");
	while(!Serial.available()){};
	CLEARSERIAL();
}

void calibMag(){
	elapsedMillis elpsdPrintTime = 0;
	CLEARSERIAL();
	slave1.imu.initCalibMagRoutine();
	while(Serial.available() <= 0){
		slave1.imu.calibMagRoutine();
		if(elpsdPrintTime > 100){
			elpsdPrintTime = 0;
			// Serial.print(slave1.imu.mx, 6);
			// Serial.print('\t');
			// Serial.println(slave1.imu.my, 6);
			Serial.print("Mag readings: "); Serial.print(slave1.imu.mx);      Serial.print("\t");Serial.print(slave1.imu.my);      Serial.print("\t"); Serial.print(slave1.imu.mz);      Serial.println();
			Serial.print("Mag Minimums: "); Serial.print(slave1.imu.MagMinX); Serial.print("\t");Serial.print(slave1.imu.MagMinY); Serial.print("\t"); Serial.print(slave1.imu.MagMinZ); Serial.println();
			Serial.print("Mag Maximums: "); Serial.print(slave1.imu.MagMaxX); Serial.print("\t");Serial.print(slave1.imu.MagMaxY); Serial.print("\t"); Serial.print(slave1.imu.MagMaxZ); Serial.println(); Serial.println();
		}
	}
	// store mag calibration in EEPROM
	slave1.imu.storeMagCalibrations();
	// calculate magnetometer calibrations
	slave1.imu.preCalculateCalibParams();
	while(Serial.available() > 0){
		Serial.read();
	}
}

int main(void){	
	Serial.begin(115200);
	pinMode(LED, OUTPUT);

	slave1.begin(115200);
	delay(200);
	slave1.lightArray.init();
	slave1.imu.init();

	//calibIMUOffset();
	//slave1.imu.calibOffset();
	//calibMag();
	while(1){   // Equivalent of the Arduino loop()
		if (Serial.available()){
			char serialCommand = Serial.read();
			CLEARSERIAL(); // make sure you only read first byte and clear everything else
			if (serialCommand == 'i'){
				Serial.println("BEGIN IMU CALIBRATION");
				Serial.println("ROTATE ROBOT");
				Serial.println("SEND ANY KEY TO BEGIN");

				// wait for key
				while(!Serial.available());
				CLEARSERIAL();

				Serial.println("\n\n\n------------------------------");
				calibMag();
			}
			else if (serialCommand == 'l'){
				Serial.println("BEGIN LIGHT SENSOR CALIBRATION");
				Serial.println("SEND ANY KEY TO BEGIN");

				// wait for key
				while(!Serial.available());
				CLEARSERIAL();

				Serial.println("\n\n\n------------------------------");
				calibLight();
			}
			else{
				Serial.println("ENTER VALID COMMAND");
			}
		}

		slave1.imu.read();
		slave1.imu.complementaryFilterBearing(1);
		bearing = -slave1.imu.yaw;
		//bearing = bearing - bearing_offset;
		TOBEARING180(bearing);
		slave1.imu.complementaryFilterBearing(0.98);
		bearing = slave1.imu.yaw;

		
		// Serial.print(slave1.imu.mx, 2);
		// Serial.print('\t');
		// Serial.print(slave1.imu.my, 2);
		// Serial.print('\t');
		// Serial.println(bearing, 2);
		
		slave1.lightArray.read();
		slave1.lightArray.getColours();
		// Serial.println();
		// PRINTARRAY(slave1.lightArray.lightData);
		// PRINTARRAY(slave1.lightArray.colours);
		// Serial.print(slave1.lightArray.armFrontSum);
		// Serial.print('\t');
		// Serial.print(slave1.lightArray.armBackSum);
		// Serial.print('\t');
		// Serial.print(slave1.lightArray.armRightSum);
		// Serial.print('\t');
		// Serial.println(slave1.lightArray.armLeftSum);
		Serial.println(bearing, 2);

		Serial.println();
		PRINTARRAY(slave1.lightArray.lightData);
		PRINTARRAY(slave1.lightArray.colours);
		
		uint8_t command = slave1.checkIfRequested();

		if (command != 255){
			SerialRequestTime = 0;
			switch(command){
	    		case SLAVE1_COMMANDS::SLAVE1_CHECK_STATUS:
	    			return slave1.peripheralStatus;
	    			break;
	    		case SLAVE1_COMMANDS::CALIB_OFFSET:
	    			calibIMUOffset();
	    			slave1.imu.calibOffset();
	    			break;
	    		case SLAVE1_COMMANDS::REQUEST_STANDARD_PACKET:
	    			commandRequestStandardPacket();
	    			break;
	    		case 255:
	    			break;
	    	}
		}
		

    	// led blinking
		if (SerialRequestTime > 100){
			// no SPI requests for past 100ms!
			// fast blink to show error
			ledBlinkTime = 50;
		}
		else{
			ledBlinkTime = 500;
		}
		if (ledElapsedTime > ledBlinkTime){
			if (ledState){
				digitalWriteFast(LED, HIGH);
			}
			else{
				digitalWriteFast(LED, LOW);
			}
			ledState = !ledState;
			ledElapsedTime = 0;
		}
	}
}

inline void commandRequestStandardPacket(){
	slave1.x = 20;
	slave1.y = 30;
	slave1.lineLocation = LINELOCATION::RECT_TOP;
	f2b.f = bearing;

	outBuffer[0] = slave1.x;
	outBuffer[1] = slave1.y;
	outBuffer[2] = f2b.b[0];
	outBuffer[3] = f2b.b[1];
	outBuffer[4] = f2b.b[2];
	outBuffer[5] = f2b.b[3];
	outBuffer[6] = slave1.lineLocation;

	slave1.sendPacket(outBuffer, 7);
}