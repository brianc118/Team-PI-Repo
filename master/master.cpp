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
#include <slaves.h>
#include <piCommon.h>
#include <PixyI2C.h>
#include <SRF08.h>
#include <fastTrig.h>
#include <PID.h>
#include <DebugUtils.h>

#define LED 13

uint32_t loopCount = 0;

/**********************************************************/
/*					     Slave1   						  */
/**********************************************************/
union float2bytes { float f; uint8_t b[sizeof(float)]; };
float2bytes f2b;
uint8_t inBuffer[22] = {0};
float bearing;
float bearing_offset;
int32_t bearing_int;

/**********************************************************/
/*                        Orbit                           */
/**********************************************************/
float orbit_k;
int16_t targetDir;
uint8_t targetVelocity;

/**********************************************************/
/*                    Face forwards                       */
/**********************************************************/
#define BEARING_KP 0.8
#define BEARING_KD 0

int32_t targetBearing = 0;
int32_t rotatationCorrection;

PID bearingPID (&bearing_int, &rotatationCorrection, &targetBearing,
	BEARING_KP, 0, BEARING_KD, -255, 255, 1000);


/**********************************************************/
/*                      Backspin                          */
/**********************************************************/
int16_t backspinSpeed = 0;

/**********************************************************/
/*                       Kicker                           */
/**********************************************************/
#define KICK_TIME 4000
#define KICK_DURATION 80

#define KICK_SIG 21
#define KICK_ANA A3

/**********************************************************/
/*					     Camera   						  */
/**********************************************************/
#define PIXY1_ADDRESS 0x54   // default i2c address

#define MIN_BLOCK_AREA   200

PixyI2C pixy1(PIXY1_ADDRESS);

int16_t blocks = 0;

int16_t goalAngle = 0;
int16_t goalAngle_rel_field = 0;
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
int16_t tsopAngle_r_goal; // tsop angle relative to goal
uint8_t tsopStrength;
uint8_t ballDistance;

uint8_t tsopData[24] = {0};


/**********************************************************/
/*					       LASER    					  */
/**********************************************************/
#define LASER_SIG A1
#define LASER_REF 152

bool ballInZone = false;

void calibIMUOffset(){
	for (int i = 0; i < 50; i++){
		Slave1.requestPacket(SLAVE1_COMMANDS::REQUEST_STANDARD_PACKET);
		Slave1.receivePacket(inBuffer, 7, true);
		
		f2b.b[0] = inBuffer[2];
		f2b.b[1] = inBuffer[3];
		f2b.b[2] = inBuffer[4];
		f2b.b[3] = inBuffer[5];
		bearing_offset += -f2b.f;
		delay(1);
	}
	bearing_offset /= 50;
}

int main(void){
	Serial.begin(115200);

	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

	SPI.setSCK(14);
	SPI.begin();	

	Slave1.begin(115200);
	Slave2.begin();
	Slave3.begin();

	pinMode(LED, OUTPUT);
	pinMode(LASER_SIG, INPUT);

	pinMode(KICK_SIG, OUTPUT);
	pinMode(KICK_ANA, INPUT);

	digitalWrite(LED, HIGH);

	delay(1000);

	calibIMUOffset();

	while(1){		
		// save some time here
		switch(loopCount % 4){
			case 0: blocks = pixy1.getBlocks();
			case 1: srfBack.getRangeIfCan(backDistance);
			case 2: srfRight.getRangeIfCan(rightDistance);
			case 3: srfLeft.getRangeIfCan(leftDistance);
		}

		/* orientation/imu */
		Slave1.requestPacket(SLAVE1_COMMANDS::REQUEST_STANDARD_PACKET);
		Slave1.receivePacket(inBuffer, 7, true);
		
		f2b.b[0] = inBuffer[2];
		f2b.b[1] = inBuffer[3];
		f2b.b[2] = inBuffer[4];
		f2b.b[3] = inBuffer[5];

		bearing = -f2b.f - bearing_offset;
		bearing_int = (int32_t)(bearing);
		TOBEARING180(bearing_int);
		TOBEARING180(bearing);

		/* end orientation/imu */

		/* tsops */
		tsopAngleByte = Slave2.getTSOP_ANGLE_BYTE();
		delayMicroseconds(50); // do i need this here?
		tsopStrength = Slave2.getTSOP_STRENGTH();
		//Slave2.getTSOPAngleStrength(tsopAngleByte, tsopStrength);

		ballDistance = 180 - tsopStrength;

		tsopAngle = tsopAngleByte * 360/255;
		TOBEARING180(tsopAngle);
		tsopAngle_r_goal = tsopAngle - goalAngle;
		TOBEARING180(tsopAngle_r_goal);

		/* end tsops */

		/* goal detection */

		goalArea = pixy1.blocks[0].width * pixy1.blocks[0].height;

		if (blocks > 0
			&& goalArea > MIN_BLOCK_AREA
			&& abs(bearing) < 90
			){
			goalDetected = true;
			goalAngle = (pixy1.blocks[0].x - 160) * 75 / 360;	
			goalAngle_rel_field = goalAngle + bearing_int;
			lGoalDetectTime = 0;		
		}
		else if (lGoalDetectTime > 100){
			// hasn't seen goal for 100ms
			goalDetected = false;
			goalAngle = 0;
		}
		/* end goal detection */

		/* face forwards */
		if (goalDetected){
			targetBearing = goalAngle_rel_field;
		}

		bearingPID.update();
		/* end face forwards */

		/* ball in zone */
		if (analogRead(LASER_SIG) < LASER_REF 
		 && abs(tsopAngle) < 30
		 && tsopStrength > 150){
			ballInZone = true;
		}
		else{
			ballInZone = false;
		}
		/* end ball in zone */

		/*movement control*/

		targetVelocity = 130;

		orbit_k = 1;

		if (tsopStrength > 156){
			orbit_k = 1.1;
		}
		else if (tsopStrength > 150){
			orbit_k = 1.05;
		}
		else if (tsopStrength > 140){
			orbit_k = 1.0;
		}
		else if (tsopStrength > 130){
			orbit_k = 1.0;
		}
		else if (tsopStrength > 120){
			orbit_k = 1;
		}
		else if (tsopStrength > 100){
			orbit_k = 1;
		}
		else if (tsopStrength > 70){
			orbit_k = 1;
		}
		else{
			targetVelocity = 0;
		}

		// if (tsopAngle > 90){
		// 	targetDir = (orbit_k * 180 - 90 + tsopAngle);
		// }
		// else if (tsopAngle < -90){
		// 	targetDir = (orbit_k * -180 + 90 + tsopAngle);
		// }
		// else{
			targetDir = orbit_k * (270 - abs(tsopAngle)) / 90 * tsopAngle;
		// }


		if (ballInZone && abs(goalAngle) < 10){
			// GO!
			targetDir = goalAngle;
			targetVelocity = 200;
		}
		

		if (targetDir < 0) targetDir += 360;
		targetDir = targetDir * 255/360;

		/* end movement control */

		/* backspin control */
		if (tsopStrength > 70){
			if (abs(targetDir) < 60){
				if (ballInZone){
					// we're going forwards and we have the ball
					// no need for too much spin
					backspinSpeed = 0;
				}
				else{
					// forwards but ball not here yet!
					backspinSpeed = 120;
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
		
		/* end backspin control */

		Slave3.moveRobot((uint8_t)targetDir, targetVelocity, rotatationCorrection);
		Slave3.moveMotorE(backspinSpeed);

		/* debugging */
		Serial.printf("%d\t%d\t%d\t%.1f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
					  micros(),
					  goalAngle_rel_field,
					  goalArea,
					  bearing,
					  backDistance,
					  rightDistance,
					  leftDistance,
					  tsopAngle,
					  ballDistance,
					  ballInZone,
					  targetDir,
					  targetVelocity,
					  rotatationCorrection);

		if(Serial.available()){
			char serialCommand = Serial.read();
			CLEARSERIAL(); // make sure you only read first byte and clear everything else
			if (serialCommand == 'i'){
				Serial.println("DEBUG INFO");
				Serial.printf("%s %s %s %s %s %s %s %s %s %s %s %s %s %s \n",
					  "micros()",
					  "goalAngle_rel_field",
					  "goalArea",
					  "bearing",
					  "backDistance",
					  "rightDistance",
					  "leftDistance",
					  "tsopAngle",
					  "ballDistance",
					  "ballInZone",
					  "targetDir",
					  "targetVelocity",
					  "rotatationCorrection");
				Serial.println("PRESS ANY KEY TO CONTINUE");
				// wait for key
				while(!Serial.available());
				CLEARSERIAL();
			}
			else{
				Serial.println("UNKNOWN COMMAND");
			}
		}
		/* end debugging */

		loopCount++;  // update loop count
	}
}