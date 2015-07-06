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

// the higher the value of COMPCONSTANT, the more gyro data is used
// COMPCONSTANT = t/(t+T) where t is the 't' is the time constant and T is the sample period
// the time constant is essentially how long we should keep the gyro data for before magnetometer data overules
#define COMPCONSTANT 0.995

#define INDEX1 1
#define INDEX2 2
#define INDEX3 4
#define INDEX4 8
#define INDEX5 16
#define INDEX6 32
#define INDEX7 64
#define INDEX8 128

union float2bytes { float f; uint8_t b[sizeof(float)]; };
float2bytes f2b;


elapsedMillis ledElapsedTime;
bool ledState = true;
uint32_t ledBlinkTime = 500;
elapsedMillis SerialRequestTime;


float bearing = 0;
float bearing_offset = 0;

uint8_t outBuffer[24] = {0};

uint8_t frontSum, backSum, rightSum, leftSum;
uint8_t lightByte = 0x00;

inline void commandRequestStandardPacket();


void calibIMUOffset(){
	bearing_offset = 0;
	for (int i = 0; i < 10000; i++){
		slave1.imu.read();
		slave1.imu.complementaryFilterBearing(COMPCONSTANT);
	}
	for (int i = 0; i < 50; i++){
		slave1.imu.read();
		slave1.imu.complementaryFilterBearing(COMPCONSTANT);
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
	CLEARSERIAL();
	Serial.println("PRESS AND KEY TO CONTINUE");
	while(!Serial.available()){};
	CLEARSERIAL();
}

void calibMagRequest(){
	Serial.println("calibMagRequest");
	slave1.imu.initCalibMagRoutine();
	bool exit = false;
	while(!exit){
		slave1.imu.calibMagRoutine();
		uint8_t c = slave1.checkIfRequested();
		switch(c){
			case SLAVE1_COMMANDS::END_CALIB_MAG:
				exit = true;
				break;
			case SLAVE1_COMMANDS::CALIB_DATA:
				commandRequestCalibData();
				Serial.println("sent data");
				break;
			case 255: break;
		}
	}
	Serial.println("ended mag calib");
	// store mag calibration in EEPROM
	slave1.imu.storeMagCalibrations();
	slave1.imu.preCalculateCalibParams();
}

extern "C" int main(void){	
	Serial.begin(115200);
	pinMode(LED, OUTPUT);

	slave1.begin(115200);
	delay(200);
	slave1.lightArray.init();
	slave1.imu.init();
	//slave1.imu.calibGyroDrift();
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
		slave1.imu.complementaryFilterBearing(COMPCONSTANT);
		bearing = -slave1.imu.yaw;
		// Serial.print(micros());
		// Serial.print('\t');
		// Serial.print(bearing, 2);

		bearing = bearing - bearing_offset;
		TOBEARING180(bearing);		

		// Serial.print('\t');
		// Serial.println(bearing, 2);
		
		// Serial.print(slave1.imu.mx, 2);
		// Serial.print('\t');
		// Serial.print(slave1.imu.my, 2);
		// Serial.print('\t');
		
		
		slave1.lightArray.read();
		slave1.lightArray.getColours();
		// Serial.println();
		PRINTARRAY(slave1.lightArray.lightData);
		PRINTARRAY(slave1.lightArray.colours);
		// Serial.print(slave1.lightArray.armFrontSum);
		// Serial.print('\t');
		// Serial.print(slave1.lightArray.armBackSum);
		// Serial.print('\t');
		// Serial.print(slave1.lightArray.armRightSum);
		// Serial.print('\t');
		// Serial.println(slave1.lightArray.armLeftSum);
		// Serial.println(bearing, 2);

		lightByte = 0x00;

		// now lightbyte is one of 16 possible values
		if (abs(bearing) <= 45){
			// we're facing forwards
			frontSum = slave1.lightArray.armFrontSum;
			backSum  = slave1.lightArray.armBackSum;
			rightSum = slave1.lightArray.armFrontSum;
			leftSum  = slave1.lightArray.armBackSum;
		}
		else if (bearing > 45 && bearing <= 135){
			// facing right
			rightSum = slave1.lightArray.armFrontSum;
			leftSum  = slave1.lightArray.armBackSum;
			backSum  = slave1.lightArray.armFrontSum;
			frontSum = slave1.lightArray.armBackSum;
		}
		else if (bearing < -45  && bearing >= -135){
			// facing left
			leftSum = slave1.lightArray.armFrontSum;
			rightSum  = slave1.lightArray.armBackSum;
			frontSum = slave1.lightArray.armFrontSum;
			backSum  = slave1.lightArray.armBackSum;
		}
		else{
			// facing back
			backSum   = slave1.lightArray.armFrontSum;
			frontSum = slave1.lightArray.armBackSum;
			leftSum   = slave1.lightArray.armFrontSum;
			rightSum = slave1.lightArray.armBackSum;
		}

		if (frontSum) lightByte = lightByte | INDEX1;
		if (backSum)  lightByte = lightByte | INDEX2;
		if (rightSum) lightByte = lightByte | INDEX3;
		if (leftSum)  lightByte = lightByte | INDEX4;
		
		Serial.println(lightByte, BIN);
		switch (lightByte){
			case 0: /*nothing*/ 
				break;
			case 1: /*front*/ 
				slave1.lineLocation = LINELOCATION::SIDE_TOP;
				break;
			case 2: /*back*/ 
				slave1.lineLocation = LINELOCATION::SIDE_BOTTOM;
				break;
			case 3: /*front back*/ 
				slave1.lineLocation = LINELOCATION::UNKNOWN;
				break;
			case 4: /*right*/ 
				slave1.lineLocation = LINELOCATION::SIDE_RIGHT;
				break;
			case 5: /*front right*/ 
				slave1.lineLocation = LINELOCATION::CORNER_TOP_LEFT;
				break;
			case 6: /*back right*/
				slave1.lineLocation = LINELOCATION::CORNER_BOTTOM_RIGHT;
				break;
			case 7: /*front back right*/ 
				slave1.lineLocation = LINELOCATION::UNKNOWN;
				break;
			case 8: /*left*/ 
				slave1.lineLocation = LINELOCATION::SIDE_LEFT;
				break;
			case 9: /*front left*/ 
				slave1.lineLocation = LINELOCATION::CORNER_TOP_LEFT;
				break;
			case 10: /*back left*/
				slave1.lineLocation = LINELOCATION::CORNER_BOTTOM_LEFT;
				break;
			case 11: /*front back left*/ 
				slave1.lineLocation = LINELOCATION::UNKNOWN;
				break;
			case 12: /*right left*/ 
				slave1.lineLocation = LINELOCATION::UNKNOWN;
				break;
			case 13: /*front right left*/ 
				slave1.lineLocation = LINELOCATION::UNKNOWN;
				break;
			case 14: /*back right left*/ 
				slave1.lineLocation = LINELOCATION::UNKNOWN;
				break;
			case 15: /*front back right left*/ 
				slave1.lineLocation = LINELOCATION::UNKNOWN;
				break;
		}
		// Serial.println();
		// PRINTARRAY(slave1.lightArray.lightData);
		// PRINTARRAY(slave1.lightArray.colours);
		
		uint8_t command = slave1.checkIfRequested();

		if (command != 255){
			SerialRequestTime = 0;
			switch(command){
	    		case SLAVE1_COMMANDS::SLAVE1_CHECK_STATUS:
	    			return slave1.peripheralStatus;
	    			break;
	    		case SLAVE1_COMMANDS::CALIB_OFFSET:
	    			calibIMUOffset();
	    			break;
	    		case SLAVE1_COMMANDS::REQUEST_STANDARD_PACKET:
	    			commandRequestStandardPacket();
	    			break;
	    		case SLAVE1_COMMANDS::CALIB_MAG:
	    			calibMagRequest(); // note that for this command another END_CALIB_MAG command must be sent to finish
	    			break;
	    		case 255:
	    			break;
	    		default:
	    			break;
	    	}
		}
		

    	// led blinking
		if (SerialRequestTime > 100){
			// no SPI requests for past 100ms!
			// fast blink to show error
			ledBlinkTime = 30;
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

inline void commandRequestCalibData(){
	f2b.f = slave1.imu.mx;
	outBuffer[0] = f2b.b[0];
	outBuffer[1] = f2b.b[1];
	outBuffer[2] = f2b.b[2];
	outBuffer[3] = f2b.b[3];
	f2b.f = slave1.imu.my;
	outBuffer[4] = f2b.b[0];
	outBuffer[5] = f2b.b[1];
	outBuffer[6] = f2b.b[2];
	outBuffer[7] = f2b.b[3];
	f2b.f = slave1.imu.MagMinX;
	outBuffer[8] = f2b.b[0];
	outBuffer[9] = f2b.b[1];
	outBuffer[10] = f2b.b[2];
	outBuffer[11] = f2b.b[3];
	f2b.f = slave1.imu.MagMaxX;
	outBuffer[12] = f2b.b[0];
	outBuffer[13] = f2b.b[1];
	outBuffer[14] = f2b.b[2];
	outBuffer[15] = f2b.b[3];
	f2b.f = slave1.imu.MagMinY;
	outBuffer[16] = f2b.b[0];
	outBuffer[17] = f2b.b[1];
	outBuffer[18] = f2b.b[2];
	outBuffer[19] = f2b.b[3];
	f2b.f = slave1.imu.MagMaxY;
	outBuffer[20] = f2b.b[0];
	outBuffer[21] = f2b.b[1];
	outBuffer[22] = f2b.b[2];
	outBuffer[23] = f2b.b[3];

	slave1.sendPacket(outBuffer, 24);
}
inline void commandRequestStandardPacket(){
	slave1.x = 20;
	slave1.y = 30;

	f2b.f = bearing;
	// Serial.print("sending");
	// Serial.println(bearing);
	outBuffer[0] = slave1.x;
	outBuffer[1] = slave1.y;
	outBuffer[2] = f2b.b[0];
	outBuffer[3] = f2b.b[1];
	outBuffer[4] = f2b.b[2];
	outBuffer[5] = f2b.b[3];
	outBuffer[6] = slave1.lineLocation;

	slave1.sendPacket(outBuffer, 7);
}