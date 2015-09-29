/*
 * master.cpp - master program for Team PI
 * 
 * Wire(0) -> capTouch -> pixies -> SRF08s -> slave2 -> slave3
 * Wire1   -> slave1
 * by Brian Chen
 * (C) Team PI 2015
 */

#include <WProgram.h>
#include <EEPROM.h>
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

// Andrew to disable the front srf again, uncomment the following line (i.e. define FDISABLED)
//#define FDISABLED

#define DEBUG_SERIAL
 
#define LED 13

// #define SUPER_SPEED 150
// #define NORMAL_SPEED 100
// #define LINE_SPEED 80
bool avoidMode = false;

uint8_t superSpeed;
uint8_t normalSpeed;
uint8_t lineSpeed;
uint8_t varSpeed_enabled;

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

uint8_t playMode = OFFENSE;
bool playModeBool = true;

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
BUTTON speedBtn(&tft, &touchedPoint);
BUTTON playModeBtn(&tft, &touchedPoint);

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
#define BOUNDARY_OFFSET 45
float dirBoundaries[2] = {-180, 180};


/**********************************************************/
/*                    Face forwards                       */
/**********************************************************/
// for speed 130, 1.0, 0.1
// for speed 170, 
#define BEARING_KP 1.2
#define BEARING_KD 0.2
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
#define KICK_MODE_KICK_TIME 6000
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

#define MIN_BLOCK_AREA 1000

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
#define SRF_FRONT_ADDRESS 0x73  // 0xE6

SRF08 srfBack(SRF_BACK_ADDRESS);
SRF08 srfRight(SRF_RIGHT_ADDRESS);
SRF08 srfLeft(SRF_LEFT_ADDRESS);
SRF08 srfFront(SRF_FRONT_ADDRESS);

int16_t backDistance, rightDistance, leftDistance, frontDistance;

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
	if (false){
		noInterrupts();
		WDOG_REFRESH = 0xA602;
		WDOG_REFRESH = 0xB480;
		interrupts();
	}
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
		//resetOtherRobotData();
		//EEPROM.write(111, xbeeConnected);		
		tx(); // try to transmit just in case the other hasn't received
	}
}

void getPlayMode(){
	if (!xbeeConnected || !otherRobot_powerOn){
		// other robot not connected or 15v not on
		//playMode = EEPROM.read(111);
		playMode = EEPROM.read(111);
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
		else if (tsopStrength > 20){
			a+=1;
		}
		if (otherRobot_tsopStrength > 150){
			b+=6;
		}
		else if (otherRobot_tsopStrength > 130){
			b+=4;
		}
		else if (otherRobot_tsopStrength > 20){
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
			else{
				if (abs(tsopAngle) < abs(otherRobot_tsopAngle)){
					playMode = OFFENSE;
				}
				else{
					playMode = DEFENSE;
				}
			}
		}
	}
	//playMode = DEFENSE;
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
		if (tsopStrength > 143){
			orbit_k = 1.0;
			orbit_l = 0.6;
		}
		else if (tsopStrength > 140){
			orbit_k = 0.9;
			orbit_l = 0.5;
		}
		else if (tsopStrength > 135){
			orbit_k = 0.8;
			orbit_l = 0.4;
		}
		else if (tsopStrength > 130){
			orbit_k = 0.8;
			orbit_l = 0.3;
		}
		else if (tsopStrength > 120){
			orbit_k = 0.75;
			orbit_l = 0.15;
		}
		else if (tsopStrength > 100){
			orbit_k = 0.6;
			orbit_l = 0.05;
		}
		else if (tsopStrength > 20){
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

		Serial.print("targetDir_r_field\t");
		Serial.print(targetDir_r_field);

		diff = targetDir_r_field - tsopAngle_r_targetBearing;
		diffComp = SIGN(tsopAngle_r_targetBearing) * minDiff - diff;

		Serial.print("orbit_k\t");
		Serial.print(orbit_k);
		Serial.print("orbit_l\t");
		Serial.print(orbit_l);
		Serial.print("mindiff\t");
		Serial.print(minDiff);
		Serial.print('\t');
		Serial.print("diffComp\t");
		Serial.print(diffComp);

		if (abs(tsopAngle_r_targetBearing) > minDiff){
			// need to cut down
			targetDir_r_field = tsopAngle_r_targetBearing + diff + orbit_l * diffComp;
			Serial.print("aaa\t");
		}
		Serial.print("targetDir_r_field\t");
		Serial.println(targetDir_r_field);
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

	// get velocity
	if (linelocation == LINELOCATION::FIELD){
		
		// Andrew if decreasing lineSpeed slows down the robot, the following 3 if cases
		// are the culprit. They are also intended to slow the robot down on the sides and back when getting
		// close to the wall, but it also works for anything in the way (i.e. other robots)
		if (leftDistance < 80 && targetDir < -80 && targetDir > -120){
			targetVelocity = lineSpeed;
		}
		else if (rightDistance < 80 && targetDir > 80 && targetDir < 120){
			targetVelocity = lineSpeed;
		}
		else if (backDistance < 80 && (targetDir < -150 || targetDir > 150)){
			targetVelocity = lineSpeed;
		}
		// Andrew pay attention to the following statement. Frontdistance is the front srf reading.
		// Change and tune the threshold of 45 as required. You may also try removing the
		// && abs(targetDir) < 60 bit, which basically means that it also checks whether
		// the robot is going to move forward between -60 and 60 degrees. If it's moving backward
		// it'll still go normal speed.
		else if (frontDistance < 45 && abs(targetDir) < 60){
			targetVelocity = lineSpeed;
		}
		else{
			if (tsopStrength > 100){
				targetVelocity = normalSpeed;
			}
			else{
				targetVelocity = superSpeed;
			}
			// back speed
			if (targetDir_r_field < -170 || targetDir_r_field > 170 && backDistance > 40){
				targetVelocity = superSpeed;
			}
		}	
	}
	else{
		targetVelocity = lineSpeed;
	}

}

void playDefense(){
	int forwardV = 0, sideV = 0;

	int backDistanceOffset;

	// if (leftDistance != 255 && rightDistance != 255){
	// 	if (leftDistance > 45 && rightDistance > 45){
	// 		if (tsopStrength > 20){
	// 		// move to ball
	// 		sideV = SIGN(tsopAngle_r_field) * 100 + 0.1*(rightDistance - leftDistance);
	// 		// in front of goal
	// 		backDistanceOffset = 25;
	// 	}
	// 	else{
		
	// 		// move to middle
	// 		backDistanceOffset = 36;
	// 		sideV = 5*(rightDistance - leftDistance);
	// 	}
	// }	

	if (leftDistance != 255 && rightDistance != 255){
		if (leftDistance > 55 && rightDistance > 55 && backDistance > 20){
			if (tsopStrength > 20){
				sideV = SIGN(tsopAngle_r_field) * 100 + 0.0*(rightDistance - leftDistance);
			}
			else{
				sideV = 10*(rightDistance - leftDistance);
			}
			backDistanceOffset = 25;
		}
		else if (leftDistance <= 55 && rightDistance > 55){
			// on left side
			sideV = 180;
			// backDistanceOffset = 36; // ints
			backDistanceOffset = 45;
		}
		else if (leftDistance > 55 && rightDistance <= 55){
			sideV = -180;
			// backDistanceOffset = 36; // ints
			backDistanceOffset = 45;
		}
		else{
			// backDistanceOffset = 25; // ints
			backDistanceOffset = 45;
			sideV = 0;
		}
	}

	if (sideV > 255){sideV = 255;}
	else if (sideV < -255){sideV = -255;}

	if (backDistance != 255){
		forwardV = 4*(backDistanceOffset - backDistance);
		if (backDistance < 20){
			forwardV = 100;
		}
	}
	if (forwardV > superSpeed){
		forwardV = superSpeed;
	}
	
	targetDir_r_field = (int16_t)(atan2(sideV, forwardV)*180/PI);
	int v_int_large = sqrt(forwardV * forwardV + sideV * sideV);
	if (v_int_large > superSpeed){
		targetVelocity = superSpeed;
	} 
	else{
		targetVelocity = v_int_large;
	}

	// override if we're on line
	switch(linelocation){
		case LINELOCATION::SIDE_LEFT:           targetDir_r_field = 90;  targetVelocity = normalSpeed; break;
		case LINELOCATION::SIDE_RIGHT:          targetDir_r_field = -90; targetVelocity = normalSpeed; break;
		case LINELOCATION::SIDE_TOP:            targetDir_r_field = 180; targetVelocity = normalSpeed; break;
		case LINELOCATION::SIDE_BOTTOM:         targetDir_r_field = 0;   targetVelocity = normalSpeed; break;
		case LINELOCATION::CORNER_TOP_RIGHT:    targetDir_r_field = 180; targetVelocity = normalSpeed; break;
		case LINELOCATION::CORNER_TOP_LEFT:     targetDir_r_field = 180; targetVelocity = normalSpeed; break;
		case LINELOCATION::CORNER_BOTTOM_RIGHT: targetDir_r_field = 0;   targetVelocity = normalSpeed; break;
		case LINELOCATION::CORNER_BOTTOM_LEFT:  targetDir_r_field = 0;   targetVelocity = normalSpeed; break;
	}		
}

void getMovement(uint8_t playMode){
	if (playMode == OFFENSE){

		if (linelocation != LINELOCATION::UNKNOWN){

			if (betweenDirBoundaries(tsopAngle_r_field) && tsopStrength > 20){
				switch(linelocation){
					case LINELOCATION::FIELD:				getOrbit(); break;
					case LINELOCATION::SIDE_LEFT:           targetDir_r_field = 90;  targetVelocity = normalSpeed; break;
					case LINELOCATION::SIDE_RIGHT:          targetDir_r_field = -90; targetVelocity = normalSpeed; break;
					case LINELOCATION::SIDE_TOP:            targetDir_r_field = 180; targetVelocity = normalSpeed; break;
					case LINELOCATION::SIDE_BOTTOM:         targetDir_r_field = 0;   targetVelocity = normalSpeed; break;
					case LINELOCATION::CORNER_TOP_RIGHT:    targetDir_r_field = 180; targetVelocity = normalSpeed; break;
					case LINELOCATION::CORNER_TOP_LEFT:     targetDir_r_field = 180; targetVelocity = normalSpeed; break;
					case LINELOCATION::CORNER_BOTTOM_RIGHT: targetDir_r_field = 0;   targetVelocity = normalSpeed; break;
					case LINELOCATION::CORNER_BOTTOM_LEFT:  targetDir_r_field = 0;   targetVelocity = normalSpeed; break;
				}
				// getOrbit();
				// if (!betweenDirBoundaries(targetDir_r_field)){
				// 	// oh no orbit won't work. But ball is in field!
				// 	if (linelocation == LINELOCATION::SIDE_LEFT ||
				// 		linelocation == LINELOCATION::SIDE_RIGHT){
				// 		// we're on the side
				// 		if (abs(tsopAngle_r_field) < 90){
				// 			// just chase ball
				// 			targetDir_r_field = tsopAngle_r_field;
				// 		}
				// 		else{
				// 			// ball behind us. Do front orbit
				// 			getOrbit(true); // front orbit
				// 			if (!betweenDirBoundaries(targetDir_r_field)){
				// 				// should never get here
				// 				targetVelocity = 0;
				// 				targetDir_r_field = 0;
				// 			}
				// 		}
				// 	}
				// 	else{
				// 		// we're in some corner => just try to chase the ball 
				// 		targetDir_r_field = tsopAngle_r_field;
				// 		if (!betweenDirBoundaries(targetDir_r_field)){
				// 			// ???
				// 			// don't know what to do in this situation
				// 			// just get out of the corner
				// 			targetVelocity = 0;
				// 			targetDir_r_field = 0;
				// 		}
				// 		switch(linelocation){
				// 			case LINELOCATION::SIDE_TOP:            targetDir_r_field = 180; targetVelocity = normalSpeed; break;
				// 			case LINELOCATION::SIDE_BOTTOM:         targetDir_r_field = 0;   targetVelocity = normalSpeed; break;
				// 			case LINELOCATION::CORNER_TOP_RIGHT:    targetDir_r_field = 180; targetVelocity = normalSpeed; break;
				// 			case LINELOCATION::CORNER_TOP_LEFT:     targetDir_r_field = 180; targetVelocity = normalSpeed; break;
				// 			case LINELOCATION::CORNER_BOTTOM_RIGHT: targetDir_r_field = 0;   targetVelocity = normalSpeed; break;
				// 			case LINELOCATION::CORNER_BOTTOM_LEFT:  targetDir_r_field = 0;   targetVelocity = normalSpeed; break;
				// 		}
				// 	}

				// }
			}
			else{
				// ball is out or ball not found
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

void speedInterface(){
	// stop moving
	Slave3.moveRobot(0, 0, 0);
	Slave3.moveMotorE(0);

	tftEnableBtn.erase();
	ltBtn.erase();
	magBtn.erase();
	kickBtn.erase();
	speedBtn.erase();
	playModeBtn.erase();

	tft.fillRect(0, 32, 320, 145, ILI9341_BLACK);

	BUTTON exitBtn(&tft, &touchedPoint);
	BUTTON spinBtn(&tft, &touchedPoint);
	BUTTON varSpeed(&tft, &touchedPoint);
	BUTTON plusNormal(&tft, &touchedPoint);
	BUTTON minusNormal(&tft, &touchedPoint);
	BUTTON plusSuper(&tft, &touchedPoint);
	BUTTON minusSuper(&tft, &touchedPoint);
	BUTTON plusLine(&tft, &touchedPoint);
	BUTTON minusLine(&tft, &touchedPoint);

	BUTTON normalText(&tft, &touchedPoint);
	BUTTON superText(&tft, &touchedPoint);
	BUTTON lineText(&tft, &touchedPoint);

	BUTTON pixyBtn(&tft, &touchedPoint);

	varSpeed.toggleBorderMode = true;
	pixyBtn.toggleBorderMode = true;

	spinBtn.setText("SPIN", 3, ILI9341_GREEN);
	varSpeed.setText("varSp", 3, ILI9341_GREEN);
	plusNormal.setText("+", 3, ILI9341_GREEN);
	minusNormal.setText("-", 3, ILI9341_GREEN);
	plusSuper.setText("+", 3, ILI9341_GREEN);
	minusSuper.setText("-", 3, ILI9341_GREEN);
	plusLine.setText("+", 3, ILI9341_GREEN);
	minusLine.setText("+", 3, ILI9341_GREEN);

	normalText.setText(String(normalSpeed), 3, ILI9341_GREEN);
	superText.setText(String(superSpeed), 3, ILI9341_GREEN);
	lineText.setText(String(lineSpeed), 3, ILI9341_GREEN);

	pixyBtn.setText("Pixy", 3, ILI9341_GREEN);

	exitBtn.setBounds(320 - 60, 0, 320, 60);
	spinBtn.setBounds(0, 180, 107, 240);
	varSpeed.setBounds(0, 120, 107, 180);


	normalText.setBounds(107, 60, 167, 120);
	superText.setBounds(167, 60, 227, 120);
	lineText.setBounds(227, 60, 287, 120);
	plusNormal.setBounds(107, 120, 167, 180);
	minusNormal.setBounds(107, 180, 167, 240);
	plusSuper.setBounds(167, 120, 227, 180);
	minusSuper.setBounds(167, 180, 227, 240);
	plusLine.setBounds(227, 120, 287, 180);
	minusLine.setBounds(227, 180, 287, 240);

	pixyBtn.setBounds(0, 60, 107, 120);

	exitBtn.setColour(ILI9341_RED);
	spinBtn.setColour(tft.color565(50,50,50));
	varSpeed.setColour(tft.color565(100,100,100));
	plusNormal.setColour(ILI9341_RED);
	minusNormal.setColour(ILI9341_YELLOW);
	plusSuper.setColour(ILI9341_RED);
	minusSuper.setColour(ILI9341_YELLOW);
	plusLine.setColour(ILI9341_RED);
	minusLine.setColour(ILI9341_YELLOW);
	normalText.setColour(ILI9341_BLACK);
	superText.setColour(ILI9341_BLACK);
	lineText.setColour(ILI9341_BLACK);
	pixyBtn.setColour(ILI9341_BLUE);

	exitBtn.draw();
	spinBtn.draw();
	varSpeed.draw();
	plusNormal.draw();
	minusNormal.draw();
	plusSuper.draw();
	minusSuper.draw();
	plusLine.draw();
	minusLine.draw();

	normalText.draw();
	superText.draw();
	lineText.draw();
	
	pixyBtn.draw();

	if (varSpeed_enabled){
		touchedPoint.x = 50;
		touchedPoint.y = 150;
		varSpeed.checkTouch();
		touchedPoint.x = 0;
		touchedPoint.y = 0;
	}

	if (pixyEnabled){
		touchedPoint.x = 50;
		touchedPoint.y = 110;
		pixyBtn.checkTouch();
		touchedPoint.x = 0;
		touchedPoint.y = 0;
	}

	do{
		kickDog();
		getTouch();
		exitBtn.checkTouch();
		spinBtn.checkTouch();
		varSpeed.checkTouch();
		plusNormal.checkTouch();
		minusNormal.checkTouch();
		plusSuper.checkTouch();
		minusSuper.checkTouch();
		plusLine.checkTouch();
		minusLine.checkTouch();
		pixyBtn.checkTouch();

		if (spinBtn.released){
			spinMode = !spinMode;
			kickBtn.enabled = false;
		}
		else if (varSpeed.released){

		}
		else if (plusNormal.released){
			if (normalSpeed + 15 > 255){
				normalSpeed = 255;
			}
			else{
				normalSpeed += 15;
			}
			normalText.setText(String(normalSpeed), 3, ILI9341_GREEN);
			normalText.draw();
		}
		else if (minusNormal.released){
			if (normalSpeed - 15 < 0){
				normalSpeed = 0;
			}
			else{
				normalSpeed -= 15;
			}
			normalText.setText(String(normalSpeed), 3, ILI9341_GREEN);
			normalText.draw();
		}
		else if (plusSuper.released){
			if (superSpeed + 15 > 255){
				superSpeed = 255;
			}
			else{
				superSpeed += 15;
			}
			superText.setText(String(superSpeed), 3, ILI9341_GREEN);
			superText.draw();
		}
		else if (minusSuper.released){
			if (superSpeed - 15 < 0){
				superSpeed = 0;
			}
			else{
				superSpeed -= 15;
			}
			superText.setText(String(superSpeed), 3, ILI9341_GREEN);
			superText.draw();
		}
		else if (plusLine.released){
			if (lineSpeed + 15 > 255){
				lineSpeed = 255;
			}
			else{
				lineSpeed += 15;
			}
			lineText.setText(String(lineSpeed), 3, ILI9341_GREEN);
			lineText.draw();
		}
		else if (minusLine.released){
			if (lineSpeed - 15 < 0){
				lineSpeed = 0;
			}
			else{
				lineSpeed -= 15;
			}
			lineText.setText(String(lineSpeed), 3, ILI9341_GREEN);
			lineText.draw();
		}
		else if (pixyBtn.released){
			pixyEnabled = !pixyEnabled;
		}
	} while(!exitBtn.released);
	EEPROM.write(10, varSpeed_enabled);
	EEPROM.write(11, normalSpeed);
	EEPROM.write(12, superSpeed);
	EEPROM.write(13, lineSpeed);
	EEPROM.write(14, pixyEnabled);
	initDebugTFT();
	drawButtons();
}

void calibLight(){
	// stop moving
	Slave3.moveRobot(0, 0, 0);
	Slave3.moveMotorE(0);

	tftEnableBtn.erase();
	ltBtn.erase();
	magBtn.erase();
	kickBtn.erase();
	speedBtn.erase();
	playModeBtn.erase();

	tft.fillRect(0, 32, 320, 145, ILI9341_BLACK);

	BUTTON exitBtn(&tft, &touchedPoint);
	BUTTON greenVBtn(&tft, &touchedPoint);
	BUTTON whiteVBtn(&tft, &touchedPoint);
	BUTTON greenHBtn(&tft, &touchedPoint);
	BUTTON whiteHBtn(&tft, &touchedPoint);
	BUTTON endBtn(&tft, &touchedPoint);

	greenVBtn.setText("G_V", 3, ILI9341_GREEN);
	whiteVBtn.setText("W_V", 3, ILI9341_GREEN);
	greenHBtn.setText("G_H", 3, ILI9341_GREEN);
	whiteHBtn.setText("W_H", 3, ILI9341_GREEN);
	endBtn.setText("END", 3, ILI9341_GREEN);

	exitBtn.setBounds(360 - 80, 0, 360, 32);
	greenVBtn.setBounds(0, 180, 64, 240);
	whiteVBtn.setBounds(62, 180, 128, 240);
	greenHBtn.setBounds(128, 180, 192, 240);
	whiteHBtn.setBounds(192, 180, 256, 240);
	endBtn.setBounds(256, 180, 320, 240);

	exitBtn.setColour(ILI9341_RED);
	greenVBtn.setColour(tft.color565(50,50,50));
	whiteVBtn.setColour(tft.color565(100,100,100));
	greenHBtn.setColour(tft.color565(50,50,50));
	whiteHBtn.setColour(tft.color565(100,100,100));
	endBtn.setColour(tft.color565(150,150,150));

	exitBtn.draw();
	greenVBtn.draw();
	whiteVBtn.draw();
	greenHBtn.draw();
	whiteHBtn.draw();
	endBtn.draw();

	tft.setTextSize(1);
	tft.setTextColor(ILI9341_WHITE);
	tft.setCursor(0, 40);

	delay(300);

	int greens = 0, whites = 0;

	do{
		kickDog();
		getTouch();
		exitBtn.checkTouch();
		greenVBtn.checkTouch();
		whiteVBtn.checkTouch();
		greenHBtn.checkTouch();
		whiteHBtn.checkTouch();
		endBtn.checkTouch();

		if (greenVBtn.released){
			tft.print("gV ");
			Slave1.requestPacket(SLAVE1_COMMANDS::CALIB_GREEN_V);
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
		else if (whiteVBtn.released){
			tft.print("wV ");
			Slave1.requestPacket(SLAVE1_COMMANDS::CALIB_WHITE_V);
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
		else if (greenHBtn.released){
			tft.print("gH ");
			Slave1.requestPacket(SLAVE1_COMMANDS::CALIB_GREEN_H);
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
		else if (whiteHBtn.released){
			tft.print("wH ");
			Slave1.requestPacket(SLAVE1_COMMANDS::CALIB_WHITE_H);
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
		kickDog();
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
		kickDog();
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
	tft.println("Calibratzion complete!");

	initDebugTFT();
	drawButtons();
}

void getSlave1Data(){
	Slave1.requestPacket(SLAVE1_COMMANDS::REQUEST_STANDARD_PACKET);
	Slave1.receivePacket(inBuffer, 7, true);
	
	linelocation = inBuffer[6];
	//linelocation = LINELOCATION::FIELD;
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
		if (tsopStrength > 20){
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
	speedBtn.setText("SPEED", 3, ILI9341_GREEN);
	playModeBtn.setText("DEF", 3, ILI9341_GREEN);

	kickBtn.toggleBorderMode = true;
	playModeBtn.toggleBorderMode = true;

	tftEnableBtn.setBounds(320 - 60, 0, 320, 60);
	magBtn.setBounds(0, 180, 107, 240);
	ltBtn.setBounds(107, 180, 214, 240);
	kickBtn.setBounds(214, 180, 320, 240);
	speedBtn.setBounds(214, 120, 320, 180);
	playModeBtn.setBounds(214, 60, 320, 120);


	tftEnableBtn.setColour(ILI9341_WHITE);
	magBtn.setColour(tft.color565(50,50,50));
	ltBtn.setColour(tft.color565(100,100,100));
	kickBtn.setColour(tft.color565(150,150,150));
	speedBtn.setColour(tft.color565(75,75,75));
	playModeBtn.setColour(tft.color565(50,50,50));

	tftEnableBtn.draw();
	magBtn.draw();
	ltBtn.draw();
	kickBtn.draw();
	speedBtn.draw();
	playModeBtn.draw();
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
	tft.println("frontDistance");
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
	tft.println(frontDistance);
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

	varSpeed_enabled = EEPROM.read(10);
	normalSpeed = EEPROM.read(11);
	superSpeed = EEPROM.read(12);
	lineSpeed = EEPROM.read(13);
	pixyEnabled = EEPROM.read(14);

	delay(500);

	kickDog();

	//XBEE.begin(9600);
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

	// srfRight.setRange(0);
	// srfLeft.setRange(0);
	// srfBack.setRange(0);

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
		switch(loopCount % 6){
			case 0: 
				if (pixyEnabled){
					blocks = pixy1.getBlocks();
				}
				break;
			case 1: srfBack.getRangeIfCan(backDistance); break;
			case 2: srfRight.getRangeIfCan(rightDistance); break;
			case 3: srfLeft.getRangeIfCan(leftDistance); break;
			case 4: 
#ifdef FDISABLED
				srfFront.getRangeIfCan(frontDistance);
#endif
			 	break;
			case 5:
				getTouch();
				tftEnableBtn.checkTouch();
				ltBtn.checkTouch();
				magBtn.checkTouch();
				kickBtn.checkTouch();
				speedBtn.checkTouch();
				playModeBtn.checkTouch();

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
					//avoidMode = !avoidMode;
					kickMode = !kickMode;
					if (kickMode){
						ltBtn.enabled = false;
						magBtn.enabled = false;
					}
				}
				else if (speedBtn.released){
					speedInterface();
				}
				else if (playModeBtn.released){
					playModeBool = !playModeBool;
					if (playModeBool){
						playMode = OFFENSE;
					}
					else{
						playMode = DEFENSE;
					}
				}

				break;
		}
		/* xbee */
		//xbeeTryTxRx();
		/* end xbee */

		/* orientation/imu */
		getSlave1Data();
		/* end orientation/imu */
#ifdef FDISABLED
		frontDistance = 255;
		switch (linelocation){
			case LINELOCATION::SIDE_TOP: frontDistance = 10; break;
			case LINELOCATION::SIDE_BOTTOM: frontDistance = 100; break;
			case LINELOCATION::CORNER_TOP_LEFT: frontDistance = 10; break;
			case LINELOCATION::CORNER_TOP_RIGHT: frontDistance = 10; break;
		}
#endif
		// location
		if (linelocation == LINELOCATION::UNKNOWN){
			// we can't tell where we are from line
			// use ultrasonics
			if (leftDistance < 40 && rightDistance < 40){
				linelocation = LINELOCATION::SIDE_TOP;
				if (backDistance < 40){
					linelocation = LINELOCATION::SIDE_BOTTOM;
				}
				if (frontDistance < 40){
					linelocation = LINELOCATION::SIDE_TOP;
				}
			}
			else if (leftDistance < 40){
				linelocation = LINELOCATION::SIDE_LEFT;
			}
			else if (rightDistance < 40){
				linelocation = LINELOCATION::SIDE_RIGHT;
			}
			else{
				linelocation = LINELOCATION::FIELD;
			}
		}
		else if (linelocation == LINELOCATION::SIDE_LEFT){
			if (leftDistance > 40 && abs(bearing_int) < 30){
				linelocation = LINELOCATION::SIDE_RIGHT;
				if (rightDistance > 40){
					linelocation = LINELOCATION::FIELD;
				}
			}
		}
		else if (linelocation == LINELOCATION::SIDE_RIGHT){
			if (rightDistance > 40 && abs(bearing_int) < 30){
				linelocation = LINELOCATION::SIDE_LEFT;
				if (leftDistance > 40){
					linelocation = LINELOCATION::FIELD;
				}
			}
		}
		else if (linelocation == LINELOCATION::SIDE_TOP){
			if (frontDistance > 40 && abs(bearing_int) < 30){
				linelocation = LINELOCATION::SIDE_BOTTOM;
				if (backDistance > 40){
					linelocation = LINELOCATION::FIELD;
				}
			}
		}
		else if (linelocation == LINELOCATION::SIDE_BOTTOM){
			if (backDistance > 40){
				linelocation = LINELOCATION::SIDE_TOP;
				if (frontDistance > 40){
					linelocation = LINELOCATION::FIELD;
				}
			}
		}
		else if (linelocation == LINELOCATION::CORNER_BOTTOM_LEFT){
			if (backDistance > 40 && abs(bearing_int) < 30){
				if (leftDistance > 40){
					linelocation = LINELOCATION::FIELD;
				}
				else{
					linelocation = LINELOCATION::SIDE_LEFT;
				}
			}
			else{
				if (leftDistance > 40){
					linelocation = LINELOCATION::SIDE_BOTTOM;
				}
			}
		}
		else if (linelocation == LINELOCATION::CORNER_BOTTOM_RIGHT){
			if (backDistance > 40 && abs(bearing_int) < 30){
				if (rightDistance > 40){
					linelocation = LINELOCATION::FIELD;
				}
				else{
					linelocation = LINELOCATION::SIDE_RIGHT;
				}
			}
			else{
				if (rightDistance > 40){
					linelocation = LINELOCATION::SIDE_TOP;
				}
			}
		}
		else if (linelocation == LINELOCATION::CORNER_TOP_LEFT){
			if (leftDistance > 40 && abs(bearing_int) < 30){
				if (frontDistance > 40){
					linelocation = LINELOCATION::FIELD;
				}
				else{
					linelocation = LINELOCATION::SIDE_TOP;
				}				
			}
			else{
				if (frontDistance > 40){
					linelocation = LINELOCATION::SIDE_LEFT;
				}
			}
		}
		else if (linelocation == LINELOCATION::CORNER_TOP_RIGHT){
			if (rightDistance > 40 && abs(bearing_int) < 30){
				if (frontDistance > 40){
					linelocation = LINELOCATION::FIELD;
				}
				else{
					linelocation = LINELOCATION::SIDE_TOP;
				}
			}
			else{
				if (frontDistance > 40){
					linelocation = LINELOCATION::SIDE_RIGHT;
				}
			}
		}

		// bearing boundaries
		switch (linelocation){
			case LINELOCATION::FIELD:
				dirBoundaries[0] = -180;
				dirBoundaries[1] = 180;
				break;
			case LINELOCATION::SIDE_LEFT:
				dirBoundaries[0] = 0 + BOUNDARY_OFFSET;
				dirBoundaries[1] = 180 - BOUNDARY_OFFSET;
				break;
			case LINELOCATION::SIDE_RIGHT:
				dirBoundaries[0] = -180 + BOUNDARY_OFFSET;
				dirBoundaries[1] = 0 - BOUNDARY_OFFSET;
				break;
			case LINELOCATION::CORNER_TOP_LEFT:
				dirBoundaries[0] = 90 + BOUNDARY_OFFSET;
				dirBoundaries[1] = 180 - BOUNDARY_OFFSET;
				break;
			case LINELOCATION::CORNER_TOP_RIGHT:
				dirBoundaries[0] = -180 + BOUNDARY_OFFSET;
				dirBoundaries[1] = -90 - BOUNDARY_OFFSET;
				break;
			case LINELOCATION::CORNER_BOTTOM_LEFT:
				dirBoundaries[0] = 0 + BOUNDARY_OFFSET;
				dirBoundaries[1] = 90 - BOUNDARY_OFFSET;
				break;
			case LINELOCATION::CORNER_BOTTOM_RIGHT:
				dirBoundaries[0] = -90 + BOUNDARY_OFFSET;
				dirBoundaries[1] = 90 - BOUNDARY_OFFSET;
				break;
			case LINELOCATION::SIDE_TOP:
				dirBoundaries[0] = 90 + BOUNDARY_OFFSET;
				dirBoundaries[1] = -90 - BOUNDARY_OFFSET;
				break;
			case LINELOCATION::SIDE_BOTTOM:
				dirBoundaries[0] = -90 + BOUNDARY_OFFSET;
				dirBoundaries[1] = 90 - BOUNDARY_OFFSET;
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
		if (playMode == OFFENSE){
			if (goalDetected){
				targetBearing = goalAngle_r_field * 1.3; // always face goals
			}
			// else if (linelocation == LINELOCATION::SIDE_LEFT){
			// 	targetBearing = 45;
			// }
			// else if (linelocation == LINELOCATION::SIDE_RIGHT){
			// 	targetBearing = -45;
			// }
			// else if (linelocation == LINELOCATION::CORNER_TOP_LEFT || linelocation == LINELOCATION::SIDE_TOP){
			// 	targetBearing = 90;
			// }
			// else if (linelocation == LINELOCATION::CORNER_TOP_RIGHT || linelocation == LINELOCATION::SIDE_BOTTOM){
			// 	targetBearing = -90;
			// }
			else{
				targetBearing = 0;
			}
		}
		else{
			if (abs(tsopAngle_r_field) < 40){
				targetBearing = 0;
			}
			else{
				targetBearing = 0;
			}
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

		//getPlayMode();
		/*movement control*/
		if (tsopStrength > 152 && abs(tsopAngle_r_field) < 30){
			getMovement(OFFENSE);
		}
		else{
			getMovement(playMode);
		}
		

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
		if (playMode == OFFENSE && linelocation == LINELOCATION::FIELD){
			if (ballInZone && goalDetected && abs(goalAngle) < 10){
				// GO!
				targetDir = goalAngle;
				targetVelocity = superSpeed;
				// kick!
				if (capChargedTime > kickTime){
					kick();
				}
			}
		}
		else{
			if (ballInZone){
				// GO!
				targetDir = goalAngle;
				targetVelocity = superSpeed;
				// kick!
				if (capChargedTime > kickTime){
					kick();
					Serial.println("kick");
				}
			}
		}
		
		checkEndKick();

		/* end movement control */

		/* backspin control */
		getBackspinSpeed();
		/* end backspin control */

		/* avoid mode */
		if (avoidMode){
			int i = 0;
			int sideV = 0;
			int forwardV = 0;
			// if (leftDistance < 45){
			// 	sideV += 100;
			// 	i++;
			// }
			// if (rightDistance < 45){
			// 	sideV -= 100;
			// 	i++;
			// }
			if (backDistance < 35){
				forwardV += 100;
				i++;
			}
			if (i > 0){
				targetDir = (int16_t)(atan2(sideV, forwardV)*180/PI);
				targetVelocity = normalSpeed;
			}
		}

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
		//serialDebug();
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
		Serial.println(linelocation);
		// if (linelocation != LINELOCATION::FIELD){

		// 	elapsedMicros iloverob = 0;
		// 	while(iloverob < 50000){
		// 		kickDog();
		// 		getSlave1Data();
		// 		getSlave2Data();
		// 		bearingPID.update();
		// 		Slave3.moveRobot(dirByte, targetVelocity, rotationCorrection);
		// 		Slave3.moveMotorE(backspinSpeed);
		// 		delay(4);
		// 	}
		// }
	}
}

// #ifdef __cplusplus
// extern "C" {
// #endif
//   void startup_early_hook() {
//     WDOG_TOVALL = (1000); // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
//     WDOG_TOVALH = 0;
//     WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
//     //WDOG_PRESC = 0; // prescaler 
//   }
// #ifdef __cplusplus
// }
// #endif