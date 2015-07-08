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

#define DEBUG_SERIAL
 
#define LED 13

#define RAMMING_SPEED 200
#define NORMAL_SPEED 150

uint32_t loopCount = 0;

// blink
elapsedMillis ledElapsedTime;

bool ledState = true;
uint32_t ledBlinkTime = 500;


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
uint8_t linelocation;

/**********************************************************/
/*                        Orbit                           */
/**********************************************************/
float orbit_l = 0.5;
float orbit_k = 1.0;
int16_t targetDir;
int16_t targetDir_r_field;
uint8_t targetVelocity;

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

#define PIXY_ENABLED

#define PIXY1_ADDRESS 0x54   // default i2c address

#define MIN_BLOCK_AREA   1500

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
#define LASER_REF 152

int laserSig = 0;

bool ballInZone = false;

inline void ledBlink(){
	// led blinking
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
	 && tsopStrength > 150){
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


	tftEnableBtn.setColour(ILI9341_RED);
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

	tft.fillRect(0, 32, 320, 145, ILI9341_BLACK);
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
	tft.println(linelocation);
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

	Slave1.begin(115200);
	Slave2.begin();
	Slave3.begin();
	Serial.println("Slaves connected");
	tft.println("Slaves connected");

	pinMode(LED, OUTPUT);
	pinMode(LASER_SIG, INPUT);

	pinMode(KICK_SIG, OUTPUT);
	pinMode(KICK_ANA, INPUT);

	digitalWrite(KICK_SIG, LOW);

	digitalWrite(LED, HIGH);

	//delay(800);
	tft.println("Calibrating IMU offset");
	calibIMUOffset();
	
	initDebugTFT();
	drawButtons();


	while(1){		
		// save some time here as reading srf08's has seen to dramatically decrease performance
		switch(loopCount % 5){
#ifdef PIXY_ENABLED
			case 0: blocks = pixy1.getBlocks(); break;
#endif
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
		/* orientation/imu */
		getSlave1Data();
		/* end orientation/imu */

		/* tsops */
		getSlave2Data();
		tsopAngle_r_field = tsopAngle + bearing_int;
		TOBEARING180(tsopAngle_r_field);
		/* end tsops */

		/* goal detection */
		getGoalData();
		/* end goal detection */

		/* face forwards */
		targetBearing = goalAngle_r_field; // always face goals
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

		/*movement control*/

		targetVelocity = NORMAL_SPEED;

		orbit_k = 1;

		if (tsopStrength > 156){
			orbit_k = 1.0;
		}
		else if (tsopStrength > 150){
			orbit_k = 0.9;
		}
		else if (tsopStrength > 140){
			orbit_k = 0.8;
		}
		else if (tsopStrength > 130){
			orbit_k = 0.7;
		}
		else if (tsopStrength > 120){
			orbit_k = 0.6;
		}
		else if (tsopStrength > 100){
			orbit_k = 0.55;
		}
		else if (tsopStrength > 70){
			orbit_k = 0.5;
		}
		else{
			targetVelocity = 0;
		}

		targetDir_r_field = (270 - abs(tsopAngle_r_targetBearing * orbit_k)) / 90 * tsopAngle_r_targetBearing * orbit_k;

		TOBEARING180(targetDir_r_field);
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
			targetVelocity = RAMMING_SPEED;
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
			backspinSpeed = 255;
		}
		Slave3.moveRobot((uint8_t)(targetDir * 255/360), targetVelocity, rotationCorrection);
		Slave3.moveMotorE(backspinSpeed);

		ledBlink();
		/* debugging */

		serialDebug();
		if (loopCount % 30 == 0){
			if (tftEnabled){
				debugTFT();
			}			
		}

		/* end debugging */

		loopCount++;  // update loop count
	}
}