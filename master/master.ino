/*
 * master.cpp - master program for Team PI
 * 
 * Wire(0) -> capTouch -> pixies -> SRF08s -> slave2 -> slave3
 * Wire1   -> slave1
 * by Brian Chen
 * (C) Team PI 2015
 */

#include <WProgram.h>
#include <i2c_t3.h>
#include <SPI.h>
#include <ILI9341_t3.h>
#include <Adafruit_FT6206.h>
#include <piGFX.h>
#include <slaves.h>
#include <piCommon.h>
#include <PixyI2C.h> // modified for i2c_t3
#include <SRF08.h>
#include <fastTrig.h>
#include <PID.h>
#include <DebugUtils.h>

//#define PHASING
#define DEBUG_SERIAL
 
#define LED 13

#define SUPER_SPEED 170
#define NORMAL_SPEED 130

uint32_t loopCount = 0;

// blink
elapsedMillis ledElapsedTime;

bool ledState = true;
uint32_t ledBlinkTime = 500;

/**********************************************************/
/*				        playMode  				     	  */
/**********************************************************/
#define OFFENSE 0
#define DEFENSE 1
#define UNDECIDED 2

uint8_t playMode = UNDECIDED;

bool powerOn = true;

/**********************************************************/
/*				        wireless   						  */
/**********************************************************/
#define XBEE_START 255
#define XBEE Serial3
#define XBEE_TIMEOUT 200000 // xbee timeout in ms

uint8_t rx_len;
uint8_t rx_packet[6];
uint8_t rx_i;
uint8_t prev_c;

elapsedMicros rx_elapsed;
bool xbeeConnected = false;

uint8_t otherRobot_playMode;
uint8_t otherRobot_tsopStrength;
int16_t otherRobot_tsopAngle;
uint8_t otherRobot_lineLocation;
bool otherRobot_powerOn;

uint8_t tx_i;

/**********************************************************/
/*					      tft   						  */
/**********************************************************/
#define TFT_DC 22
#define TFT_CS 20

bool tftEnabled = true;

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, 255, 11, 14, 12);

Adafruit_FT6206 ctp = Adafruit_FT6206();

bool touched = false;
POINT touchedPoint;

BUTTON tftEnableBtn(&tft, &touchedPoint);
BUTTON magBtn(&tft, &touchedPoint);
BUTTON ltBtn(&tft, &touchedPoint);
BUTTON kickBtn(&tft, &touchedPoint);
BUTTON spinBtn(&tft, &touchedPoint);

bool kickMode = false;

/**********************************************************/
/*					     Slave1   						  */
/**********************************************************/
union float2bytes { float f; uint8_t b[sizeof(float)]; };
float2bytes f2b;
uint8_t inBuffer[24] = {0};
float bearing;
float bearing_offset;
int32_t bearing_int;
uint8_t linelocation, linelocation_prev;

/**********************************************************/
/*                        Orbit                           */
/**********************************************************/
float orbit_l = 0.5;
float orbit_k = 1.0;
int16_t targetDir;
int16_t lDir;
int16_t targetDir_r_field;
uint8_t targetVelocity;

bool inFrontOrbitMode = false;

/**********************************************************/
float dirBoundaries[2] = {-180, 180};


/**********************************************************/
/*                    Face forwards                       */
/**********************************************************/
// for speed 130, 1.0, 0.1
// for speed 170, 
#define BEARING_KP 0.7
#define BEARING_KD 0.1
#define ROTATIONCORRECTION_MAX 70

int32_t targetBearing = 0;
int32_t rotationCorrection;

PID bearingPID (&bearing_int, &rotationCorrection, &targetBearing,
	BEARING_KP, 0, BEARING_KD, -ROTATIONCORRECTION_MAX, ROTATIONCORRECTION_MAX, 1000);


/**********************************************************/
/*                      Backspin                          */
/**********************************************************/
bool spinMode = false; // spinMode triggers constant backspin

int16_t backspinSpeed = 0;

/**********************************************************/
/*                       Kicker                           */
/**********************************************************/
#define KICK_TIME 4000
#define KICK_MODE_KICK_TIME 15000
#define KICK_DELAY 30
#define KICK_DURATION 90

#define KICK_SIG 21
#define KICK_ANA A3

uint16_t kickTime;

bool kicking = false;
bool kickDelayComplete = false;
elapsedMillis capChargedTime = 0;

/**********************************************************/
/*					     Camera   						  */
/**********************************************************/

bool pixyEnabled = true;

#define PIXY1_ADDRESS 0x54   // default i2c address

#define MIN_BLOCK_AREA   1000

PixyI2C pixy1(PIXY1_ADDRESS);

int16_t blocks = 0;

int16_t goalAngle = 0;
int16_t goalAngle_r_field = 0;
uint16_t goalArea = 0;

elapsedMillis lGoalDetectTime = 0;

bool goalDetected = false;

/**********************************************************/
/*					     Ultrasonics					  */
/**********************************************************/
#define SRF_BACK_ADDRESS  0x70  // 0xE0
#define SRF_RIGHT_ADDRESS 0x71  // 0xE2
#define SRF_LEFT_ADDRESS  0x72  // 0xE4

SRF08 srfBack(SRF_BACK_ADDRESS);
SRF08 srfRight(SRF_RIGHT_ADDRESS);
SRF08 srfLeft(SRF_LEFT_ADDRESS);

int16_t backDistance, rightDistance, leftDistance;

/**********************************************************/
/*					       TSOPS    					  */
/**********************************************************/

uint8_t tsopAngleByte;
int16_t tsopAngle;
int16_t tsopAngle_r_field;
int16_t tsopAngle_r_targetBearing;
uint8_t tsopStrength;
uint8_t ballDistance;

uint8_t tsopData[24] = {0};


/**********************************************************/
/*					       LASER    					  */
/**********************************************************/
#define LASER_SIG A1
#define LASER_REF 950

int laserSig = 0;

bool ballInZone = false;

void kickDog() {
	noInterrupts();
	WDOG_REFRESH = 0xA602;
	WDOG_REFRESH = 0xB480;
	interrupts();
}

void resetOtherRobotData(){
	otherRobot_playMode = DEFENSE;
	otherRobot_tsopStrength = 0;
	otherRobot_tsopAngle = 0;
	otherRobot_lineLocation = LINELOCATION::UNKNOWN;
	otherRobot_powerOn = false;
}

void rx(){
	otherRobot_playMode = rx_packet[0];
	otherRobot_tsopStrength = rx_packet[1];
	otherRobot_tsopAngle  = (rx_packet[2] << 8) | rx_packet[3];
	otherRobot_lineLocation = rx_packet[4];
	otherRobot_powerOn = rx_packet[5];
	// Serial.print(micros()); Serial.print('\t');
	// Serial.print(otherRobot_playMode); Serial.print('\t');
	// Serial.print(otherRobot_tsopStrength); Serial.print('\t');
	// Serial.print(otherRobot_tsopAngle); Serial.print('\t');
	// Serial.print(otherRobot_lineLocation); Serial.print('\t');
	// Serial.println(otherRobot_powerOn);
}

void tx(){
	XBEE.write(XBEE_START); 
	XBEE.write(XBEE_START); 
	XBEE.write(playMode); 
	XBEE.write(tsopStrength); 
	XBEE.write(highByte(tsopAngle)); 
	XBEE.write(lowByte(tsopAngle)); 
	XBEE.write(linelocation); 
	XBEE.write(powerOn); 
}

void xbeeTryTxRx(){
	rx_len = XBEE.available();
	if(rx_len){
		while(XBEE.available()){
			uint8_t c = XBEE.read();
			//Serial.print(rx_i);
			if(c == XBEE_START && prev_c == XBEE_START){
				rx_i = 0;
			}
			else if(rx_i <= 5){
				rx_packet[rx_i] = c;
				rx_i++;
			}
			if(rx_i == 6){
				rx();
				tx();
				rx_elapsed = 0;
				xbeeConnected = true;
				rx_i = 7;
			}
			prev_c = c;
		}
	}

	if(rx_elapsed >= XBEE_TIMEOUT){
		rx_elapsed = 0;
		xbeeConnected = false;
		resetOtherRobotData();
		tx(); // try to transmit just in case the other hasn't received
	}
}

void getPlayMode(){
	if (!xbeeConnected || !otherRobot_powerOn){
		// other robot not connected or 15v not on
		playMode = OFFENSE;
	}
	else{
		int a = 0, b = 0;
		// other robot connected.
		// first layer compares tsop strength
		if (tsopStrength > 150){
			a+=6;
		}
		else if (tsopStrength > 130){
			a+=4;
		}
		else if (tsopStrength > 70){
			a+=1;
		}
		if (otherRobot_tsopStrength > 150){
			b+=6;
		}
		else if (otherRobot_tsopStrength > 130){
			b+=4;
		}
		else if (otherRobot_tsopStrength > 70){
			b+=1;
		}
		if (a > b){
			playMode = OFFENSE;
		}
		else if (a < b){
			playMode = DEFENSE;
		}
		else{
			// first check that they're not both not seeing the ball.
			// if they both can't see the ball they are both forced to be defense
			if (tsopStrength == 0){
				playMode == DEFENSE;
			}
			// similar strength now look at angle
			if (abs(DIFF180(tsopAngle, otherRobot_tsopAngle)) < 10){
				// too close to call, consider strength mainly
				if (tsopStrength > otherRobot_tsopStrength){
					playMode = OFFENSE;
				}
				else{
					playMode = DEFENSE;
				}
			}
		}		
	}
}



bool betweenDirBoundaries(float a){
	return isBetween(a, dirBoundaries[0], dirBoundaries[1]);
}

void getOrbit(bool frontOrbit = false){
	// regardless of input, if inFrontOrbitMode, then must be front orbit
	// check if front orbit complete
	if (frontOrbit){
		// check if you can actually front orbit
		frontOrbit = abs(tsopAngle_r_targetBearing) > 90;
	}
	if (inFrontOrbitMode && abs(tsopAngle_r_targetBearing) > 90){
		inFrontOrbitMode = false;
	}

	if (!frontOrbit && !inFrontOrbitMode){
		targetVelocity = NORMAL_SPEED;

		if (tsopStrength > 148){
			orbit_k = 1.0;
			orbit_l = 0.5;
		}
		else if (tsopStrength > 140){
			orbit_k = 0.9;
			orbit_l = 0.4;
		}
		else if (tsopStrength > 135){
			orbit_k = 0.8;
			orbit_l = 0.3;
		}
		else if (tsopStrength > 130){
			orbit_k = 0.7;
			orbit_l = 0.2;
		}
		else if (tsopStrength > 120){
			orbit_k = 0.6;
			orbit_l = 0.1;
		}
		else if (tsopStrength > 100){
			targetVelocity = SUPER_SPEED;
			orbit_k = 0.55;
			orbit_l = 0.05;
		}
		else if (tsopStrength > 70){
			targetVelocity = SUPER_SPEED;
			orbit_k = 0.5;
			orbit_l = 0;
		}
		else{
			targetVelocity = 0;
		}
		float minDiff = (270 - 90 * orbit_k) / 90 * 90 * orbit_k - 90; // min diff after 90 degrees
		float diff;
		float diffComp; // complement of difference

		targetDir_r_field = (270 - abs(tsopAngle_r_targetBearing * orbit_k)) / 90 * tsopAngle_r_targetBearing * orbit_k;

		diff = targetDir_r_field - tsopAngle_r_targetBearing;
		diffComp = SIGN(tsopAngle_r_targetBearing) * minDiff - diff;

		if (abs(diffComp) > minDiff){
			// need to cut down
			targetDir_r_field = tsopAngle_r_targetBearing + diff + orbit_l * diffComp;
		}
	}
	else{
		if (tsopAngle_r_targetBearing > 0){
			targetDir_r_field = tsopAngle_r_field - 90;
		}
		else{
			targetDir_r_field = tsopAngle_r_field + 90;
		}
		inFrontOrbitMode = true;
	}
	TOBEARING180(targetDir_r_field);

	if (targetDir_r_field < -170 || targetDir_r_field > 170){
		targetVelocity = SUPER_SPEED;
	}
}

void playDefense(){
	int forwardV = 0, sideV = 0;
	
	if (tsopStrength > 70){
		// move to ball
		if (tsopAngle_r_field > 5){
			// ball on right.
			sideV = 100;
		}
		else if (tsopAngle_r_field < -5){
			sideV = -100;
		}
		else{
			sideV = 0;
		}
	}
	else{
		if (leftDistance != 255 && rightDistance != 255){
			// move to middle
			sideV = 2*(rightDistance - leftDistance);
		}
	}	

	if (backDistance != 255){
		if (backDistance > 30){
			forwardV = -100;
		}
		else if (backDistance < 15){
			forwardV = 100;
		}
	}
	targetDir_r_field = (int16_t)(atan2(sideV, forwardV)*180/PI);
	targetVelocity = NORMAL_SPEED;

	// override if we're on line
	switch(linelocation){
		case LINELOCATION::SIDE_LEFT:           targetDir_r_field = 90;  targetVelocity = NORMAL_SPEED; break;
		case LINELOCATION::SIDE_RIGHT:          targetDir_r_field = -90; targetVelocity = NORMAL_SPEED; break;
		case LINELOCATION::SIDE_TOP:            targetDir_r_field = 180; targetVelocity = NORMAL_SPEED; break;
		case LINELOCATION::SIDE_BOTTOM:         targetDir_r_field = 0;   targetVelocity = NORMAL_SPEED; break;
		case LINELOCATION::CORNER_TOP_RIGHT:    targetDir_r_field = 180; targetVelocity = NORMAL_SPEED; break;
		case LINELOCATION::CORNER_TOP_LEFT:     targetDir_r_field = 180; targetVelocity = NORMAL_SPEED; break;
		case LINELOCATION::CORNER_BOTTOM_RIGHT: targetDir_r_field = 0;   targetVelocity = NORMAL_SPEED; break;
		case LINELOCATION::CORNER_BOTTOM_LEFT:  targetDir_r_field = 0;   targetVelocity = NORMAL_SPEED; break;
	}		
}

void getMovement(uint8_t playMode){
	if (playMode == OFFENSE){
		if (linelocation != LINELOCATION::UNKNOWN){
			if (betweenDirBoundaries(tsopAngle_r_field)){
				getOrbit();
				if (!betweenDirBoundaries(targetDir_r_field)){
					// oh no orbit won't work. But ball is in field!
					if (linelocation == LINELOCATION::SIDE_LEFT ||
						linelocation == LINELOCATION::SIDE_RIGHT){
						// we're on the side
						if (abs(tsopAngle_r_field) < 90){
							// just chase ball
							targetDir_r_field = tsopAngle_r_field;
						}
						else{
							// ball behind us. Do front orbit
							getOrbit(true); // front orbit
							if (!betweenDirBoundaries(targetDir_r_field)){
								// should never get here
								targetVelocity = 0;
								targetDir_r_field = 0;
							}
						}
					}
					else{
						// we're in some corner => just try to chase the ball 
						targetDir_r_field = tsopAngle_r_field;
						if (!betweenDirBoundaries(targetDir_r_field)){
							// ???
							// don't know what to do in this situation
							// just get out of the corner
						}
						switch(linelocation){
							case LINELOCATION::SIDE_TOP:            targetDir_r_field = 180; targetVelocity = NORMAL_SPEED; break;
							case LINELOCATION::SIDE_BOTTOM:         targetDir_r_field = 0;   targetVelocity = NORMAL_SPEED; break;
							case LINELOCATION::CORNER_TOP_RIGHT:    targetDir_r_field = 180; targetVelocity = NORMAL_SPEED; break;
							case LINELOCATION::CORNER_TOP_LEFT:     targetDir_r_field = 180; targetVelocity = NORMAL_SPEED; break;
							case LINELOCATION::CORNER_BOTTOM_RIGHT: targetDir_r_field = 0;   targetVelocity = NORMAL_SPEED; break;
							case LINELOCATION::CORNER_BOTTOM_LEFT:  targetDir_r_field = 0;   targetVelocity = NORMAL_SPEED; break;
						}		
					}
				}
			}
			else{
				// ball is out
				playDefense();	
			}
		}
		else{
			// unknown
			targetVelocity = 0;
			targetDir = 0;
		}
	}
	else if (playMode == DEFENSE){
		playDefense();	
	}
	else{
		// ???
	}
}

void ledBlink(){
	// led blinking
	if (ledElapsedTime > ledBlinkTime){
		if (ledState)   digitalWriteFast(LED, HIGH);
		else     		digitalWriteFast(LED, LOW);
		ledState = !ledState;
		ledElapsedTime = 0;
	}
}

void kick(){
	if (!kicking){
		// not yet kicking start kick
		kicking = true;
		capChargedTime = 0;
		Serial.print(millis());
		Serial.println("KICK_MOTORS");
	}	
}

void checkEndKick(){
	if (kicking){
		if (!kickDelayComplete && capChargedTime > KICK_DELAY){
			kickDelayComplete = true;
			digitalWriteFast(KICK_SIG, HIGH);
			capChargedTime = 0; // now effectively discharge time
			Serial.print(millis());
			Serial.println("KICK");
		}
		else if (kickDelayComplete){
			// currently actually kicking
			if (capChargedTime >= KICK_DURATION){
				// end kick
				kicking = false;
				kickDelayComplete = false;
				digitalWriteFast(KICK_SIG, LOW);
				capChargedTime = 0;
				//Serial.print(analogRead(KICK_ANA));
				Serial.print(millis());
				Serial.println("END");
			}
		}
	}
	else{
		digitalWriteFast(KICK_SIG, LOW);
	}
}

void calibIMUOffset(){
	Slave1.requestPacket(SLAVE1_COMMANDS::CALIB_OFFSET);
}

void calibLight(){
	// stop moving
	Slave3.moveRobot(0, 0, 0);
	Slave3.moveMotorE(0);

	tftEnableBtn.erase();
	ltBtn.erase();
	magBtn.erase();
	kickBtn.erase();
	spinBtn.erase();

	tft.fillRect(0, 32, 320, 145, ILI9341_BLACK);

	BUTTON exitBtn(&tft, &touchedPoint);
	BUTTON greenBtn(&tft, &touchedPoint);
	BUTTON whiteBtn(&tft, &touchedPoint);
	BUTTON endBtn(&tft, &touchedPoint);

	greenBtn.setText("GREEN", 3, ILI9341_GREEN);
	whiteBtn.setText("WHITE", 3, ILI9341_GREEN);
	endBtn.setText("END", 3, ILI9341_GREEN);

	exitBtn.setBounds(360 - 80, 0, 360, 32);
	greenBtn.setBounds(0, 180, 107, 240);
	whiteBtn.setBounds(107, 180, 214, 240);
	endBtn.setBounds(214, 180, 320, 240);

	exitBtn.setColour(ILI9341_RED);
	greenBtn.setColour(tft.color565(50,50,50));
	whiteBtn.setColour(tft.color565(100,100,100));
	endBtn.setColour(tft.color565(150,150,150));

	exitBtn.draw();
	greenBtn.draw();
	whiteBtn.draw();
	endBtn.draw();

	tft.setTextSize(1);
	tft.setTextColor(ILI9341_WHITE);
	tft.setCursor(0, 40);

	delay(300);

	int greens = 0, whites = 0;

	do{
		getTouch();
		exitBtn.checkTouch();
		greenBtn.checkTouch();
		whiteBtn.checkTouch();
		endBtn.checkTouch();

		if (greenBtn.released){
			tft.print("g ");
			Slave1.requestPacket(SLAVE1_COMMANDS::CALIB_GREEN);
			greens++;
			delay(2);
			Slave1.requestPacket(SLAVE1_COMMANDS::LIGHT_DATA_GREEN);
			Slave1.receivePacket(inBuffer, 16, true);

			for (int i = 0; i < 16; i++){
				if (inBuffer[i] < 10){
					tft.print("  ");
				}
				else if (inBuffer[i] <= 100){
					tft.print(" ");
				}
				tft.print(inBuffer[i]);
				tft.print(" ");
			}
			tft.println();
		}
		else if (whiteBtn.released){
			tft.print("w ");
			Slave1.requestPacket(SLAVE1_COMMANDS::CALIB_WHITE);
			whites++;
			delay(2);
			Slave1.requestPacket(SLAVE1_COMMANDS::LIGHT_DATA_WHITE);
			Slave1.receivePacket(inBuffer, 16, true);		
			for (int i = 0; i < 16; i++){
				if (inBuffer[i] < 10){
					tft.print("  ");
				}
				else if (inBuffer[i] <= 100){
					tft.print(" ");
				}
				tft.print(inBuffer[i]);
				tft.print(" ");
			}
			tft.println();
		}
		else if (endBtn.released){
			if (greens && whites){
				Slave1.requestPacket(SLAVE1_COMMANDS::END_CALIB_LIGHT);
				tft.println();
				tft.println("FINISHED CALIB");
				tft.print("  ");
				delay(2);
				Slave1.requestPacket(SLAVE1_COMMANDS::LIGHT_DATA_REFS);
				Slave1.receivePacket(inBuffer, 16, true);		
				for (int i = 0; i < 16; i++){
					if (inBuffer[i] < 10){
						tft.print("  ");
					}
					else if (inBuffer[i] <= 100){
						tft.print(" ");
					}
					if (inBuffer[i] == 255){
						tft.setTextColor(ILI9341_RED);
					}
					else{
						tft.setTextColor(ILI9341_WHITE);
					}
					tft.print(inBuffer[i]);
					tft.print(" ");
				}
				tft.println();
			}
			else{
				if (!greens){
					tft.println("NEED GREEN");
				}
				if (!whites){
					tft.println("NEED WHITE");
				}
			}
		}
	} while(!exitBtn.released);
	initDebugTFT();
	drawButtons();
}

void calibMag(){
	// stop moving
	Slave3.moveRobot(0, 0, 0);
	Slave3.moveMotorE(0);

	tft.fillRect(0, 32, 320, 145, ILI9341_BLACK);

	Slave1.requestPacket(SLAVE1_COMMANDS::CALIB_MAG);
	// now we've entered calibration mode
	tft.setCursor(0, 50);
	tft.setTextSize(2);
	tft.setTextColor(ILI9341_RED);
	tft.println("entered calibration mode");
	tft.println("tap anywhere on screen to");
	tft.println("finish calibration");
	tft.println();

	touched = false;
	while(!touched){
		getTouch();
		Slave1.requestPacket(SLAVE1_COMMANDS::CALIB_DATA);
		Slave1.receivePacket(inBuffer, 24, true);

		tft.fillRect(0, 80, 320, 180, ILI9341_BLACK);
		tft.setCursor(0, 80);

		f2b.b[0] = inBuffer[0];
		f2b.b[1] = inBuffer[1];
		f2b.b[2] = inBuffer[2];
		f2b.b[3] = inBuffer[3];

		tft.print(f2b.f); tft.print(" ");

		f2b.b[0] = inBuffer[4];
		f2b.b[1] = inBuffer[5];
		f2b.b[2] = inBuffer[6];
		f2b.b[3] = inBuffer[7];

		tft.print(f2b.f); tft.print("\n");

		f2b.b[0] = inBuffer[8];
		f2b.b[1] = inBuffer[9];
		f2b.b[2] = inBuffer[10];
		f2b.b[3] = inBuffer[11];

		tft.print(f2b.f); tft.print(" ");

		f2b.b[0] = inBuffer[12];
		f2b.b[1] = inBuffer[13];
		f2b.b[2] = inBuffer[14];
		f2b.b[3] = inBuffer[15];

		tft.print(f2b.f); tft.print("\n");

		f2b.b[0] = inBuffer[16];
		f2b.b[1] = inBuffer[17];
		f2b.b[2] = inBuffer[18];
		f2b.b[3] = inBuffer[19];

		tft.print(f2b.f); tft.print(" ");

		f2b.b[0] = inBuffer[20];
		f2b.b[1] = inBuffer[21];
		f2b.b[2] = inBuffer[22];
		f2b.b[3] = inBuffer[23];

		tft.print(f2b.f); tft.print("\n");

		delay(20);
	}
	// exit when touched
	Slave1.requestPacket(SLAVE1_COMMANDS::END_CALIB_MAG);
	tft.println();
	tft.println("Calibration complete!");

	initDebugTFT();
	drawButtons();
}

void getSlave1Data(){
	Slave1.requestPacket(SLAVE1_COMMANDS::REQUEST_STANDARD_PACKET);
	Slave1.receivePacket(inBuffer, 7, true);
	
	linelocation = inBuffer[6];

	f2b.b[0] = inBuffer[2];
	f2b.b[1] = inBuffer[3];
	f2b.b[2] = inBuffer[4];
	f2b.b[3] = inBuffer[5];

	bearing = f2b.f;
	bearing_int = (int32_t)(bearing);
	TOBEARING180(bearing_int);
	TOBEARING180(bearing);
}

void getSlave2Data(){
	Slave2.getTSOPAngleStrength(tsopAngle, tsopStrength);
	TOBEARING180(tsopAngle);
}

void getGoalData(){
	goalArea = pixy1.blocks[0].width * pixy1.blocks[0].height;

	if (blocks > 0 && blocks < 1000
		&& goalArea > MIN_BLOCK_AREA
		&& abs(bearing) < 90
		){
		goalDetected = true;
		goalAngle = (pixy1.blocks[0].x - 160) * 75 / 360;	
		lGoalDetectTime = 0;		
		// Serial.print("goaldetected");
		// Serial.println(blocks);
	}
	else if (lGoalDetectTime > 100){
		// hasn't seen goal for 100ms
		goalDetected = false;
		goalAngle = -bearing_int;
	}

	goalAngle_r_field = goalAngle + bearing_int;
}

void getBackspinSpeed(){
	if (!kicking){
		if (tsopStrength > 70){
			if (abs(targetDir) < 60){
				if (ballInZone){
					// we're going forwards and we have the ball
					// no need for too much spin
					backspinSpeed = 120;
				}
				else{
					// forwards but ball not here yet!
					backspinSpeed = 255;
				}
			}
			else{
				if (ballInZone){
					// we're not going forwards but we have the ball
					// best to spin a bit more as we need to guide the ball more
					backspinSpeed = 150;
				}
				else{
					// most common. Should only spin when ball is in front
					if (abs(tsopAngle) < 60){
						backspinSpeed = 255;
					}
					else{
						// no chance of getting ball any time soon
						backspinSpeed = 0;
					}
				}
			}
		}
		else{
			// no ball detected!
			backspinSpeed = 0;
		}
	}
	else{
		backspinSpeed = -255;
	}
}

void checkBallInZone(){
	laserSig = analogRead(LASER_SIG);
	if (laserSig < LASER_REF 
	 && abs(tsopAngle) < 30
	 && tsopStrength > 140){
		ballInZone = true;
	}
	else{
		ballInZone = false;
	}
}

void getTouch(){
	touchedPoint = ctp.getPoint(); 
	touched = (touchedPoint.x != 0 || touchedPoint.y != 0);
	if (touched){
		int x, y;
		x = touchedPoint.y;
		y = 240 - touchedPoint.x;
		touchedPoint.x = x;
		touchedPoint.y = y;
	}
}

void drawPiLogo(){
	tft.setRotation(3);
	tft.fillScreen(ILI9341_BLACK);
	tft.setTextColor(ILI9341_WHITE);
	tft.setTextSize(4);
	tft.println("Team PI");
	tft.setTextSize(1);
}

void drawButtons(){
	magBtn.setText("MAG", 3, ILI9341_GREEN);
	ltBtn.setText("LT", 3, ILI9341_GREEN);
	kickBtn.setText("KICK", 3, ILI9341_GREEN);
	spinBtn.setText("SPIN", 3, ILI9341_GREEN);

	kickBtn.toggleBorderMode = true;
	spinBtn.toggleBorderMode = true;

	tftEnableBtn.setBounds(320 - 60, 0, 320, 60);
	magBtn.setBounds(0, 180, 107, 240);
	ltBtn.setBounds(107, 180, 214, 240);
	kickBtn.setBounds(214, 180, 320, 240);
	spinBtn.setBounds(214, 120, 320, 180);


	tftEnableBtn.setColour(ILI9341_WHITE);
	magBtn.setColour(tft.color565(50,50,50));
	ltBtn.setColour(tft.color565(100,100,100));
	kickBtn.setColour(tft.color565(150,150,150));
	spinBtn.setColour(tft.color565(75,75,75));

	tftEnableBtn.draw();
	magBtn.draw();
	ltBtn.draw();
	kickBtn.draw();
	spinBtn.draw();
}

void initDebugTFT(){
	// tft.color565(50,50,50)
	tft.setTextSize(1);

	tft.fillRect(0, 32, 320, 208, ILI9341_BLACK);
	//tft.fillScreen(ILI9341_BLACK);
	tft.setCursor(0, 40);
	tft.setTextColor(ILI9341_RED);
	tft.println("micros");
	tft.setTextColor(ILI9341_YELLOW);
	tft.println("goalAngle_r_field");
	tft.println("goalArea");
	tft.setTextColor(ILI9341_MAGENTA);
	tft.println("backDistance");
	tft.println("rightDistance");
	tft.println("leftDistance");
	tft.setTextColor(ILI9341_WHITE);
	tft.println("tsopAngle");
	tft.println("tsopAngle_r_targetBearing");
	tft.println("tsopStrength");
	tft.setTextColor(ILI9341_GREEN);
	tft.println("ballInZone");
	tft.println("targetDir");
	tft.println("targetDir_r_field");
	tft.println("targetVelocity");
	tft.println("bearing");
	tft.println("rotationCorrection");
	tft.setTextColor(ILI9341_CYAN);
	tft.println("linelocation");
	tft.setTextColor(ILI9341_WHITE);
	tft.println("xbeeConnected");
}
void debugTFT(){
	tft.invertDisplay(false);
	tft.fillRect(160, 32, 54, 145, ILI9341_BLACK);
	tft.setCursor(160, 40);
	tft.x_offset = 160;

	tft.setTextColor(ILI9341_RED);
	tft.setTextSize(1);
	tft.println(micros());
	tft.setTextColor(ILI9341_YELLOW);
	tft.println(goalAngle_r_field);
	tft.println(goalArea);
	tft.setTextColor(ILI9341_MAGENTA);
	tft.println(backDistance);
	tft.println(rightDistance);
	tft.println(leftDistance);
	tft.setTextColor(ILI9341_WHITE);
	tft.println(tsopAngle);
	tft.println(tsopAngle_r_targetBearing);
	tft.println(tsopStrength);
	tft.setTextColor(ILI9341_GREEN);
	tft.print(ballInZone); tft.print(" "); tft.println(laserSig);
	tft.println(targetDir);
	tft.println(targetDir_r_field);
	tft.println(targetVelocity);
	tft.println(bearing);
	tft.println(rotationCorrection);
	tft.setTextColor(ILI9341_CYAN);
	switch (linelocation){
		case LINELOCATION::FIELD: tft.println("f"); break;
		case LINELOCATION::UNKNOWN: tft.println("u"); break;
		case LINELOCATION::CORNER_BOTTOM_LEFT: tft.println("c_bl"); break;
		case LINELOCATION::CORNER_BOTTOM_RIGHT: tft.println("c_br"); break;
		case LINELOCATION::CORNER_TOP_LEFT: tft.println("c_tl"); break;
		case LINELOCATION::CORNER_TOP_RIGHT: tft.println("c_tr"); break;
		case LINELOCATION::SIDE_TOP: tft.println("t"); break;
		case LINELOCATION::SIDE_BOTTOM: tft.println("b"); break;
		case LINELOCATION::SIDE_LEFT: tft.println("l"); break;
		case LINELOCATION::SIDE_RIGHT: tft.println("r");  break;
	}
	tft.setTextColor(ILI9341_WHITE);
	tft.println(xbeeConnected);
	tft.x_offset = 0;
}

void serialDebug(){
#ifdef DEBUG_SERIAL
	Serial.printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.0f\t%d\n",
				  micros(),
				  goalAngle_r_field,
				  goalArea,
				  backDistance,
				  rightDistance,
				  leftDistance,
				  tsopAngle,
				  tsopAngle_r_targetBearing,
				  tsopStrength,
				  ballInZone,
				  targetDir,
				  targetDir_r_field,
				  targetVelocity,
				  bearing,
				  rotationCorrection);

	if(Serial.available()){
		char serialCommand = Serial.read();
		CLEARSERIAL(); // make sure you only read first byte and clear everything else
		if (serialCommand == 'i'){
			Serial.println("DEBUG INFO");
			Serial.printf("%s %s %s %s %s %s %s %s %s %s %s %s %s %s \n",
				  "micros()",
				  "goalAngle_r_field",
				  "goalArea",
				  "bearing",
				  "backDistance",
				  "rightDistance",
				  "leftDistance",
				  "tsopAngle",
				  "tsopStrength",
				  "ballInZone",
				  "targetDir",
				  "targetVelocity",
				  "rotationCorrection");
			Serial.println("PRESS ANY KEY TO CONTINUE");
			// wait for key
			while(!Serial.available());
			CLEARSERIAL();
		}
		else{
			Serial.println("UNKNOWN COMMAND");
		}
	}
#endif
}

extern "C" int main(void){	
	Serial.begin(115200);

	// begin i2c at 400kHz
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
	Wire.setDefaultTimeout(1000);

	SPI.setSCK(14);
	SPI.begin();	

	delay(1000);

	kickDog();

	XBEE.begin(57600);
	tft.begin();
	drawPiLogo();

	if (!ctp.begin(30)) {  // pass in 'sensitivity' coefficient
		Serial.println("Couldn't start FT6206 touchscreen controller");
		tft.println("Couldn't start FT6206 touchscreen controller");
	}
	else{
		Serial.println("Couldn't start FT6206 touchscreen controller");
		tft.println("FT6206 connected");
	}

	delay(450);

	Slave1.begin(115200);
	Slave2.begin();
	Slave3.begin();
	Serial.println("Slaves connected");
	tft.println("Slaves connected");

	Wire.beginTransmission(PIXY1_ADDRESS);
	if (Wire.endTransmission() != 0){
		pixyEnabled = false;
		Serial.println("pixy disconnected");
		tft.println("pixy disconnected");
	}
	else{
		Serial.println("pixy connected");
		tft.println("pixy connected");
	}

	delay(50);

	pinMode(LED, OUTPUT);
	pinMode(LASER_SIG, INPUT);

	pinMode(KICK_SIG, OUTPUT);
	pinMode(KICK_ANA, INPUT);

	digitalWrite(KICK_SIG, LOW);

	digitalWrite(LED, HIGH);

	//delay(800);
	tft.println("Calibrating IMU offset");
	//calibIMUOffset();
	
	initDebugTFT();
	drawButtons();

	tx();
	while(1){		
		kickDog();
		// save some time here as reading srf08's has seen to dramatically decrease performance
		switch(loopCount % 5){
			case 0: 
				if (pixyEnabled){
					blocks = pixy1.getBlocks();
				}
				break;
			case 1: srfBack.getRangeIfCan(backDistance); break;
			case 2: srfRight.getRangeIfCan(rightDistance); break;
			case 3: srfLeft.getRangeIfCan(leftDistance); break;
			case 4:
				getTouch();
				tftEnableBtn.checkTouch();
				ltBtn.checkTouch();
				magBtn.checkTouch();
				kickBtn.checkTouch();
				spinBtn.checkTouch();

				if (tftEnableBtn.released){
					// toggle display and touch
					tftEnabled = !tftEnabled;
					ltBtn.enabled = !ltBtn.enabled;
					magBtn.enabled = !magBtn.enabled;
					kickBtn.enabled = !kickBtn.enabled;
				}
				else if (magBtn.released){
					calibMag();
				}
				else if (ltBtn.released){
					calibLight();
				}
				else if (kickBtn.released){
					kickMode = !kickMode;
					if (kickMode){
						ltBtn.enabled = false;
						magBtn.enabled = false;
					}
				}
				else if (spinBtn.released){
					spinMode = !spinMode;
					kickBtn.enabled = false;
				}

				break;
		}
		/* xbee */
		xbeeTryTxRx();
		/* end xbee */

		/* orientation/imu */
		getSlave1Data();
		/* end orientation/imu */

		// location
		if (linelocation == LINELOCATION::UNKNOWN){
			// we can't tell where we are from line
			// use ultrasonics
			if (leftDistance < 35 && rightDistance < 35){
				linelocation = LINELOCATION::SIDE_TOP;
				if (backDistance < 35){
					linelocation = LINELOCATION::SIDE_BOTTOM;
				}
			}
			else if (leftDistance < 35){
				linelocation = LINELOCATION::SIDE_LEFT;
			}
			else if (rightDistance < 35){
				linelocation = LINELOCATION::SIDE_RIGHT;
			}
			else{
				linelocation = LINELOCATION::UNKNOWN;
			}
		}

		// bearing boundaries
		switch (linelocation){
			case LINELOCATION::FIELD:
				dirBoundaries[0] = -180;
				dirBoundaries[1] = 180;
				break;
			case LINELOCATION::SIDE_LEFT:
				dirBoundaries[0] = -180;
				dirBoundaries[1] = 0;
				break;
			case LINELOCATION::SIDE_RIGHT:
				dirBoundaries[0] = 0;
				dirBoundaries[1] = 180;
				break;
			case LINELOCATION::CORNER_TOP_LEFT:
				dirBoundaries[0] = 90;
				dirBoundaries[1] = 180;
				break;
			case LINELOCATION::CORNER_TOP_RIGHT:
				dirBoundaries[0] = -180;
				dirBoundaries[1] = -90;
				break;
			case LINELOCATION::CORNER_BOTTOM_LEFT:
				dirBoundaries[0] = 0;
				dirBoundaries[1] = 90;
				break;
			case LINELOCATION::CORNER_BOTTOM_RIGHT:
				dirBoundaries[0] = -90;
				dirBoundaries[1] = 90;
				break;
			case LINELOCATION::SIDE_TOP:
				dirBoundaries[0] = 90;
				dirBoundaries[1] = -90;
				break;
			case LINELOCATION::SIDE_BOTTOM:
				dirBoundaries[0] = -90;
				dirBoundaries[1] = 90;
				break;
			case LINELOCATION::UNKNOWN:
				dirBoundaries[0] = -180;
				dirBoundaries[1] = 180;
				break;
		}

		/* tsops */
		getSlave2Data();
		tsopAngle_r_field = tsopAngle + bearing_int;
		TOBEARING180(tsopAngle_r_field);
		/* end tsops */

		/* goal detection */
		if (pixyEnabled){
			getGoalData();
		}
		/* end goal detection */

		/* face forwards */
		if (goalDetected){
			targetBearing = goalAngle_r_field * 1.3; // always face goals
		}
		else if (linelocation == LINELOCATION::SIDE_LEFT){
			targetBearing = 45;
		}
		else if (linelocation == LINELOCATION::SIDE_RIGHT){
			targetBearing = -45;
		}
		else if (linelocation == LINELOCATION::CORNER_TOP_LEFT || linelocation == LINELOCATION::SIDE_TOP){
			targetBearing = 90;
		}
		else if (linelocation == LINELOCATION::CORNER_TOP_RIGHT || linelocation == LINELOCATION::SIDE_BOTTOM){
			targetBearing = -90;
		}
		else{
			targetBearing = 0;
		}
		
		tsopAngle_r_targetBearing = tsopAngle_r_field - targetBearing; // target bearing is relative to field
		TOBEARING180(tsopAngle_r_targetBearing);

		// only try to face forwards if solenoid isn't in use
		// otherwise solenoid interfers a lot with magnetometer
		if (!kicking){
			bearingPID.update();
		}
		/* end face forwards */

		/* ball in zone */
		checkBallInZone();
		/* end ball in zone */

		getPlayMode();

		/*movement control*/
		getMovement(playMode);

		targetDir = targetDir_r_field - bearing_int;
		TOBEARING180(targetDir);
		
		if (kickMode){
			goalDetected = true;
			ballInZone = true;
			goalAngle = 0;
			kickTime = KICK_MODE_KICK_TIME;
		}
		else{
			kickTime = KICK_TIME;
		}
		
		if (ballInZone && goalDetected && abs(goalAngle) < 10){
			// GO!
			targetDir = goalAngle;
			targetVelocity = SUPER_SPEED;
			// kick!
			if (capChargedTime > kickTime){
				kick();
			}
		}
		checkEndKick();

		/* end movement control */

		/* backspin control */
		getBackspinSpeed();
		/* end backspin control */

		if (kickMode){
			backspinSpeed = 0;
			targetVelocity = 0;
			rotationCorrection = 0;
		}
		if (spinMode){
			targetVelocity = 0;
			backspinSpeed = 255;
			rotationCorrection = 0;
		}

		/* phasing */
#ifdef PHASING
		if (DIFF180(lDir, targetDir) > 1){
			targetDir = lDir + 1;
		}
		else if (DIFF180(lDir, targetDir) < -1){
			targetDir = lDir - 1;
		}
		TOBEARING180(targetDir);

		lDir = targetDir;
#endif
		/* end phasing */

		uint8_t dirByte = (uint8_t)(targetDir * 255/360);
		Slave3.moveRobot(dirByte, targetVelocity, rotationCorrection);
		Slave3.moveMotorE(backspinSpeed);

		

		ledBlink();
		/* debugging */
		serialDebug();
		if (loopCount % 30 == 0){
			if (tftEnabled){
				debugTFT();
				if (playMode == OFFENSE){
					tftEnableBtn.setColour(ILI9341_RED);
				}
				else if (playMode == DEFENSE){
					tftEnableBtn.setColour(ILI9341_BLUE);
				}
				else{
					tftEnableBtn.setColour(ILI9341_WHITE);
				}
				tftEnableBtn.draw(false);
			}
		}

		/* end debugging */

		loopCount++;  // update loop count
		linelocation_prev = linelocation;
	}
}

#ifdef __cplusplus
extern "C" {
#endif
  void startup_early_hook() {
    WDOG_TOVALL = (1000); // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
    WDOG_TOVALH = 0;
    WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
    //WDOG_PRESC = 0; // prescaler 
  }
#ifdef __cplusplus
}
#endif