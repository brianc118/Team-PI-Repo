/*
 * slave3.cpp - Team Pi Slave3 Code
 * 
 * 
 * Slave 3 runs as an SPI slave and is dedicated to motor speed control of
 * 4 motors (PID control from encoder feedback) and the 5th motor for
 * backspin. 2 encoders are decoded using the mk20dx256's quadrature decoder interface
 * whilst another 2 are decoded in software.
 *
 * Please set clock to 96MHz for best chance of working!
 * 
 * ENCODER DETAILS
 * 
 * ENCXA 	3	PTA12 	28	Input 	FTM1 	7
 * ENCXB 	4	PTA13 	29	Input 	FTM1 	7
 * ENCYA 	32	PTB18 	41	Input 	FTM2 	6
 * ENCYB 	25	PTB19 	42	Input 	FTM2 	6
 * 
 * Many thanks to tlb who contributed hardware quadrature code specifically
 * for the mk20dx128/mk20dx256, btmcmahan who made his SPI slave library
 * available for the Teensy 3.x.
 * 
 * Dependencies:
 *   https://github.com/btmcmahan/Teensy-3.0-SPI-Master---Slave (modified to compile)
 *   http://forum.pjrc.com/threads/26803-Hardware-Quadrature-Code-for-Teensy-3-x
 * 
 * All other non core libraries are written by me.
 * 
 * by Brian Chen
 * Originally authored in November 2014
 * (C) 2015 Team Pi 
 */

#include <WProgram.h>
#include <t3spi.h>			// spi slave library
#include <QuadDecode_def.h> // quad decoder library (hardware)
#include <pwmMotor.h> 		// motor control library
#include <piCommon.h>	   // common header for all teensies. Share definitions of commands etc.
#include <fastTrig.h>
#include <omnidrive.h> 		// library for omnidirectional movement
#include <PID.h>

#define MOVEMOTOR(input, d, b, pwm){	\
	if (input < 0){					 \
		digitalWriteFast(d, LOW);	  \
		digitalWriteFast(b, LOW);	   \
		analogWrite(pwm, abs(input));   \
	} else if(input > 0){			   \
		digitalWriteFast(d, HIGH);	   \
		digitalWriteFast(b,LOW);		\
		analogWrite(pwm,abs(input));	\
	} else{							 \
		digitalWrite(b,LOW);		   \
		analogWrite(pwm,abs(input)); }} \


// general
uint8_t status = 0;	 // status of program/slavD
#define LED 13

/**SPI***********************************************************/
#define SPI_SCK		SCK
#define SPI_MOSI	MOSI
#define SPI_MISO	MISO
//#define SPI_CS 		ALT_CS0
#define SPI_CS 		CS0

T3SPI SPI;

volatile uint8_t dataExpected = 0;
volatile uint8_t command;
volatile uint8_t spiBuff[5] = {0};

volatile uint32_t spiRequestCount = 0;
volatile uint32_t spiRequestCount_prev = 0;

/**MOTORS********************************************************/
#define MAX_RPM		12400 // theoretical max rpm of Maxon motor according to datasheet

#define PWM_FREQ 	16000

#define MT_A_PWM	5
#define MT_B_PWM	6
#define MT_C_PWM	9
#define MT_D_PWM	10
#define MT_E_PWM 	23

#define MT_A_DIR	0
#define MT_B_DIR	1
#define MT_C_DIR	15
#define MT_D_DIR	20
#define MT_E_DIR 	33

#define MT_A_BRK	7
#define MT_B_BRK	8
#define MT_C_BRK	16
#define MT_D_BRK	17
#define MT_E_BRK 	24

#define MT_A_CS		A10
#define MT_B_CS		A11
#define MT_C_CS		A12
#define MT_D_CS		A13
#define MT_E_CS 	A20
// MT_E_CS is also connected on the master teensy

/**CONTROL********************************************************/
#define KP			1
#define KI			0
#define KD			0

// PMOTOR motorA(MT_A_PWM, MT_A_DIR, MT_A_PWMA_BRK, true, MT_A_CS);
// PMOTOR motorB(MT_B_PWM, MT_B_DIR, MT_B_BRK, true, MT_B_CS);
// PMOTOR motorC(MT_C_PWM, MT_C_DIR, MT_C_BRK, true, MT_C_CS);
// PMOTOR motorD(MT_D_PWM, MT_D_DIR, MT_D_BRK, true, MT_D_CS);
// PMOTOR motorE(MT_E_PWM, MT_E_DIR, MT_E_BRK, true, MT_E_CS);

volatile uint8_t csA = 0, csB = 0, csC = 0, csD = 0, csE = 0;

int32_t pA = 0, pB = 0, pC = 0, pD = 0, pE = 0;

omnidrive omni(&pA, &pB, &pC, &pD);

volatile bool updateMove = false;
volatile int16_t dir = 10;
volatile uint8_t velocity = 10;
volatile int8_t rotation_velocity = 10;

/**ENCODER********************************************************/
#define CPR		   64


/**DECODER********************************************************/
//#define HARDWAREDECODE // comment out to disable all hardware decoding

#define SOFTWARE_UPDATE_INTERVAL 18 // time between each update in us 18
#define ENCXA	   3  //|
#define ENCXB	   4  //| Use built in hardware quadrature interface
#define ENCYA	   19 //|
#define ENCYB	   18 //|
#define ENCZA	   32
#define ENCZB	   25
#define ENCEA	   21
#define ENCEB	   22

#ifdef HARDWAREDECODE
QuadDecode<1> xPosn;	// Template using FTM1
QuadDecode<2> yPosn;	// Template using FTM2
#endif

volatile int32_t rtA = 0, rtB = 0, rtC = 0, rtD = 0;	// Realtime values
		 int32_t prA = 0, prB = 0, prC = 0, prD = 0;	// Previous values

volatile int16_t temp16;

int32_t vA = 0, vB = 0, vC = 0, vD = 0;	// velocity that really is just elapsed counts / s
int32_t vA_targ, vB_targ, vC_targ, vD_targ; // target velocities (really in elapsed counts / ss)

IntervalTimer encoderUpdater;
/* since there are 4 phases a quadrature encoder goes through, there are 16 combinations, many of which
   are invalid (and hence = 0) */
int32_t enc_TAB[]= { 0, 1, -1, 0,
 					-1, 0,  0, 1,
 					 1, 0,  0,-1,
 					 0,-1,  1, 0 };

volatile uint32_t encState = 0, lEncState = 0;
volatile uint32_t encIndex;

// PID (integer)

#define KP 1
#define KI 0
#define KD 0

int32_t pidOutA, pidOutB, pidOutC, pidOutD;
PID pidA(&vA, &pidOutA, &vA_targ, KP, KI, KD, -255, 255, 0); // last argument is update interval. Set as 0 to update any time you want
PID pidB(&vB, &pidOutB, &vB_targ, KP, KI, KD, -255, 255, 0);
PID pidC(&vC, &pidOutC, &vC_targ, KP, KI, KD, -255, 255, 0);
PID pidD(&vD, &pidOutD, &vD_targ, KP, KI, KD, -255, 255, 0);

volatile int32_t channelCount = 0, encoderUpdateCount = 0;


void encoderUpdate();

elapsedMillis ledElapsedTime;

bool ledState = true;
uint32_t ledBlinkTime = 500;

extern "C" int main(void){
	CORE_PIN33_CONFIG = 0;  // completely disables the pin
	Serial.begin(115200);

	pinMode(LED, OUTPUT);

	// motorA.begin(PWM_FREQ);
	// motorB.begin(PWM_FREQ);
	// motorC.begin(PWM_FREQ);
	// motorD.begin(PWM_FREQ);
	// motorE.begin(PWM_FREQ);

	pinMode(MT_A_DIR, OUTPUT);
	pinMode(MT_B_DIR, OUTPUT);
	pinMode(MT_C_DIR, OUTPUT);
	pinMode(MT_D_DIR, OUTPUT);
	pinMode(MT_E_DIR, OUTPUT);

	pinMode(MT_A_BRK, OUTPUT);
	pinMode(MT_B_BRK, OUTPUT);
	pinMode(MT_C_BRK, OUTPUT);
	pinMode(MT_D_BRK, OUTPUT);
	pinMode(MT_E_BRK, OUTPUT);

	pinMode(MT_A_PWM, OUTPUT);
	pinMode(MT_B_PWM, OUTPUT);
	pinMode(MT_C_PWM, OUTPUT);
	pinMode(MT_D_PWM, OUTPUT);
	pinMode(MT_E_PWM, OUTPUT);

	pinMode(MT_A_CS, INPUT);
	pinMode(MT_B_CS, INPUT);
	pinMode(MT_C_CS, INPUT);
	pinMode(MT_D_CS, INPUT);
	pinMode(MT_E_CS, INPUT);

	pinMode(ENCXA, INPUT);
	pinMode(ENCXB, INPUT);
	pinMode(ENCYA, INPUT);
	pinMode(ENCYB, INPUT);
	pinMode(ENCZA, INPUT);
	pinMode(ENCZB, INPUT);
	pinMode(ENCEA, INPUT);
	pinMode(ENCEB, INPUT);

	analogWriteFrequency(MT_A_PWM, PWM_FREQ);
	analogWriteFrequency(MT_B_PWM, PWM_FREQ);
	analogWriteFrequency(MT_C_PWM, PWM_FREQ);
	analogWriteFrequency(MT_D_PWM, PWM_FREQ);
	analogWriteFrequency(MT_E_PWM, PWM_FREQ);

	// MOVEMOTOR(0, MT_A_DIR, MT_A_BRK, MT_A_PWM);
	// MOVEMOTOR(0, MT_B_DIR, MT_B_BRK, MT_B_PWM);
	// MOVEMOTOR(0, MT_C_DIR, MT_C_BRK, MT_C_PWM);
	// MOVEMOTOR(0, MT_D_DIR, MT_D_BRK, MT_D_PWM);
	// MOVEMOTOR(0, MT_E_DIR, MT_E_BRK, MT_E_PWM);
	
	// for fastest speeds configure 8 bit analog reads and no averaging
	analogReadResolution(8);
	analogReadAveraging(32);

	ledBlinkTime = 0;

	noInterrupts();

	SPI.begin_SLAVE(ALT_SCK, MOSI, MISO, ALT_CS0);
  	SPI.setCTAR_SLAVE(8, SPI_MODE0);
  	NVIC_SET_PRIORITY(IRQ_SPI0, 127); // set priority
 	NVIC_ENABLE_IRQ(IRQ_SPI0);	// SPI interrupt 	

#ifdef HARDWAREDECODE
	xPosn.start();		// Start Quad Decode position count
	yPosn.start();		// Start Quad Decode position count
#endif
	encoderUpdater.priority(127);
	encoderUpdater.begin(encoderUpdate, SOFTWARE_UPDATE_INTERVAL);

	interrupts();
	// main loop
	while(1){
		delay(1);
		//Serial.println(csE);
		// for (int i = -255; i < 255; i++){
		// 	MOVEMOTOR(i, MT_A_DIR, MT_A_BRK, MT_A_PWM)
		// 	delay(10);
		// }
		// MOVEMOTOR(0, MT_A_DIR, MT_A_BRK, MT_A_PWM)
		// delay(1000);
		// for (int i = -255; i < 255; i++){
		// 	MOVEMOTOR(i, MT_B_DIR, MT_B_BRK, MT_B_PWM);
		// 	delay(10);
		// }
		// MOVEMOTOR(0, MT_B_DIR, MT_B_BRK, MT_B_PWM)
		// delay(1000);
		// for (int i = -255; i < 255; i++){
		// 	MOVEMOTOR(i, MT_C_DIR, MT_C_BRK, MT_C_PWM);
		// 	delay(10);
		// }
		// MOVEMOTOR(0, MT_C_DIR, MT_C_BRK, MT_C_PWM)
		// delay(1000);
		// for (int i = -255; i < 255; i++){
		// 	MOVEMOTOR(i, MT_D_DIR, MT_D_BRK, MT_D_PWM);
		// 	delay(10);
		// }
		// MOVEMOTOR(0, MT_D_DIR, MT_D_BRK, MT_D_PWM)
		// delay(1000);
		// MOVEMOTOR(10, MT_A_DIR, MT_A_BRK, MT_A_PWM);
		// MOVEMOTOR(10, MT_B_DIR, MT_B_BRK, MT_B_PWM);
		// MOVEMOTOR(10, MT_C_DIR, MT_C_BRK, MT_C_PWM);
		// MOVEMOTOR(10, MT_D_DIR, MT_D_BRK, MT_D_PWM);
		// MOVEMOTOR(10, MT_E_DIR, MT_E_BRK, MT_E_PWM);
		
		// if (updateMove){
		// 	Serial.print("move");
		// 	// if (dir = 0){
		// 	// 	// something wrong
		// Serial.print(dir);
		// Serial.print('\t');
		// Serial.print(velocity);
		// Serial.print('\t');
		// Serial.print(rotation_velocity);
		// Serial.println();
		// 	// 	Serial.println(millis());
		// 	// }
		// 	// else{
		// 	omni.move(dir, velocity, rotation_velocity);
			
		// 	updateMove = false;
		// }
		if (dataExpected == 0){
			Serial.print(dir);
			Serial.print('\t');
			Serial.print(velocity);
			Serial.print('\t');
			Serial.println(rotation_velocity);
			// e = 0;
			// Serial.print(rtA);
			// Serial.print('\t');
			// Serial.print(rtB);
			// Serial.print('\t');
			// Serial.print(rtC);
			// Serial.print('\t');
			// Serial.print(rtD);
			// Serial.print('\t');
			// Serial.print(vA);
			// Serial.print('\t');
			// Serial.print(vB);
			// Serial.print('\t');
			// Serial.print(vC);
			// Serial.print('\t');
			// Serial.println(vD);
			
		}
		// Serial.print(dir);
		// Serial.print('\t');
		// Serial.print(velocity);
		// Serial.print('\t');
		// Serial.println(rotation_velocity);
	}
}

// Interrupt Service Routine to handle incoming data
void spi0_isr(){
	noInterrupts();
	if (dataExpected == 0){
		command = SPI0_POPR;
		Serial.println(command);
		CLEARARRAY(spiBuff);

		switch(command){
			case SLAVE3_COMMANDS::SLAVE3_CHECK_STATUS: SPI0_PUSHR_SLAVE = status;            dataExpected = 0;  break;
			case SLAVE3_COMMANDS::MOVE1:			   SPI0_PUSHR_SLAVE = csA;	             dataExpected = 1;  break;
			case SLAVE3_COMMANDS::MOVE2:			   SPI0_PUSHR_SLAVE = csB;	             dataExpected = 1;  break;
			case SLAVE3_COMMANDS::MOVE3:			   SPI0_PUSHR_SLAVE = csC;	             dataExpected = 1;  break;
			case SLAVE3_COMMANDS::MOVE4:			   SPI0_PUSHR_SLAVE = csD;	             dataExpected = 1;  break;  
			case SLAVE3_COMMANDS::MOVE5:			   SPI0_PUSHR_SLAVE = csE;	             dataExpected = 1;  break;  
			case SLAVE3_COMMANDS::CSENSE1:			   SPI0_PUSHR_SLAVE = csA;	             dataExpected = 0;  break;
			case SLAVE3_COMMANDS::CSENSE2:			   SPI0_PUSHR_SLAVE = csB;	             dataExpected = 0;  break;
			case SLAVE3_COMMANDS::CSENSE3:			   SPI0_PUSHR_SLAVE = csC;	             dataExpected = 0;  break;
			case SLAVE3_COMMANDS::CSENSE4:			   SPI0_PUSHR_SLAVE = csD;	             dataExpected = 0;  break;
			case SLAVE3_COMMANDS::CSENSE5:			   SPI0_PUSHR_SLAVE = csE;	             dataExpected = 0;  break;
			case SLAVE3_COMMANDS::V1:				   SPI0_PUSHR_SLAVE = (vA >> 8) & 0xff;	 dataExpected = 1;  break;	
			case SLAVE3_COMMANDS::V2:				   SPI0_PUSHR_SLAVE = (vB >> 8) & 0xff;	 dataExpected = 1;  break;	
			case SLAVE3_COMMANDS::V3:				   SPI0_PUSHR_SLAVE = (vC >> 8) & 0xff;	 dataExpected = 1;  break;	
			case SLAVE3_COMMANDS::V4:				   SPI0_PUSHR_SLAVE = (vD >> 8) & 0xff;	 dataExpected = 1;  break;	
			case SLAVE3_COMMANDS::BRAKE1:
				pA = 0;
				digitalWriteFast(MT_A_BRK, HIGH);	         
				dataExpected = 0;  
				break;
			case SLAVE3_COMMANDS::BRAKE2:
				pB = 0;
				digitalWriteFast(MT_B_BRK, HIGH);	         
				dataExpected = 0;  
				break;
			case SLAVE3_COMMANDS::BRAKE3:
				pC = 0;
				digitalWriteFast(MT_C_BRK, HIGH);
				dataExpected = 0;
				break;
			case SLAVE3_COMMANDS::BRAKE4:
				pD = 0;
				digitalWriteFast(MT_D_BRK, HIGH);
				dataExpected = 0;  
				break;
			case SLAVE3_COMMANDS::BRAKE5:
				pE = 0;
				digitalWriteFast(MT_E_BRK, HIGH);
				dataExpected = 0;
				break;
			case SLAVE3_COMMANDS::BRAKEALL:
				pA = 0; pB = 0; pC = 0; pD = 0; pE = 0;
				digitalWriteFast(MT_A_BRK, HIGH);
				digitalWriteFast(MT_B_BRK, HIGH);
				digitalWriteFast(MT_C_BRK, HIGH);
				digitalWriteFast(MT_D_BRK, HIGH);
				digitalWriteFast(MT_E_BRK, HIGH);
				dataExpected = 0;
				break;
			case SLAVE3_COMMANDS::MOVE: 			   SPI0_PUSHR_SLAVE = 1;	         dataExpected = 1;  break;
		}
	}
	else if (dataExpected == 1){
		// uint8_t temp = SPI0_POPR;
		// Serial.print("\tb");
		// Serial.println(SPI0_POPR);
		switch(command){ // command is really the previous command. 
			case SLAVE3_COMMANDS::MOVE1: spiBuff[0] = SPI0_POPR; dataExpected = 2;  break;
			case SLAVE3_COMMANDS::MOVE2: spiBuff[0] = SPI0_POPR; dataExpected = 2;  break;
			case SLAVE3_COMMANDS::MOVE3: spiBuff[0] = SPI0_POPR; dataExpected = 2;  break;
			case SLAVE3_COMMANDS::MOVE4: spiBuff[0] = SPI0_POPR; dataExpected = 2;  break;
			case SLAVE3_COMMANDS::MOVE5: spiBuff[0] = SPI0_POPR; dataExpected = 2;  break;
			case SLAVE3_COMMANDS::V1:	 SPI0_PUSHR_SLAVE = vA & 0xff;   dataExpected = 0;  break;
			case SLAVE3_COMMANDS::V2:	 SPI0_PUSHR_SLAVE = vB & 0xff;	 dataExpected = 0;  break;
			case SLAVE3_COMMANDS::V3:	 SPI0_PUSHR_SLAVE = vC & 0xff;	 dataExpected = 0;  break;
			case SLAVE3_COMMANDS::V4:	 SPI0_PUSHR_SLAVE = vD & 0xff;	 dataExpected = 0;  break;
			case SLAVE3_COMMANDS::MOVE:  dir = SPI0_POPR * 360/255; SPI0_PUSHR_SLAVE = 2; dataExpected = 2; break;
		}		
	}
	else if (dataExpected == 2){
		// Serial.print("\tc");
		// Serial.println(SPI0_POPR);
		int16_t p_;
		spiBuff[1] = SPI0_POPR; 
		
		switch(command){ // command is really the previous command.
			case SLAVE3_COMMANDS::MOVE1: p_ = (spiBuff[0] << 8) | spiBuff[1];  pA = (int32_t)p_;   dataExpected = 0; break;
			case SLAVE3_COMMANDS::MOVE2: p_ = (spiBuff[0] << 8) | spiBuff[1];  pB = (int32_t)p_;   dataExpected = 0; break;
			case SLAVE3_COMMANDS::MOVE3: p_ = (spiBuff[0] << 8) | spiBuff[1];  pC = (int32_t)p_;   dataExpected = 0; break;
			case SLAVE3_COMMANDS::MOVE4: p_ = (spiBuff[0] << 8) | spiBuff[1];  pD = (int32_t)p_;   dataExpected = 0; break;
			case SLAVE3_COMMANDS::MOVE5: p_ = (spiBuff[0] << 8) | spiBuff[1];  pE = (int32_t)p_;   dataExpected = 0; break;
			case SLAVE3_COMMANDS::MOVE: velocity = spiBuff[1]; SPI0_PUSHR_SLAVE = 3; dataExpected = 3; break;
		}		
	}
	else if (dataExpected == 3){
		// Serial.println(SPI0_POPR);		
		// Serial.print("\td");
		switch(command){ // command is really the previous command.
			case SLAVE3_COMMANDS::MOVE:
				rotation_velocity = SPI0_POPR; 
				//seems like teensy can't cope with move() command in this interrupt
				omni.move(dir, velocity, rotation_velocity);
				// dir = 195;
				// velocity = 50;
				// rotation_velocity = 0;
				// SPI0_PUSHR_SLAVE = 4;
				updateMove = true;
				dataExpected = 0;
				break;
		}	
	}
	SPI0_SR |= SPI_SR_RFDF;
	//delay(50);	
	spiRequestCount++;
	//Serial.println();
	interrupts();
}

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

// this routine is called via an interrupt to update the encoders
void encoderUpdate(){
	// noInterrupts();

#ifdef HARDWAREDECODE
	// software decode Z and E channels
	encState = (digitalReadFast(ENCZB)<<9) | (digitalReadFast(ENCZA)<<8) | (digitalReadFast(ENCEB)<<13) | (digitalReadFast(ENCEA)<<12);
	// hardware decode X and Y channels
	rtA = xPosn.calcPosn();
	rtB = yPosn.calcPosn();
#else
	encState = (digitalReadFast(ENCXB)<<1) | (digitalReadFast(ENCXA))	 | (digitalReadFast(ENCYB)<<5)  | (digitalReadFast(ENCYA)<<4) 
			 | (digitalReadFast(ENCZB)<<9) | (digitalReadFast(ENCZA)<<8) | (digitalReadFast(ENCEB)<<13) | (digitalReadFast(ENCEA)<<12);
#endif

	encIndex = encState | (lEncState << 2);

	/* byte   4	3	2	1
			  lB   lA   B	A */


	// if anything the following 4 lines are the best to optimise
#ifndef HARDWAREDECODE
	rtA += enc_TAB[ (encIndex >> 0)  & 0x0f]; // we are only using 4 bytes per encoder
	rtB += enc_TAB[ (encIndex >> 4)  & 0x0f];
#endif
	rtC += enc_TAB[ (encIndex >> 8)  & 0x0f];
	rtD += enc_TAB[ (encIndex >> 12) & 0x0f];

	lEncState = encState;

	// pA = 50;
	// pB = 50;
	// pC = 50;
	// pD = 50;
	// pE = 100;
	// encoder decoding stuff done. Begin PID
	if (encoderUpdateCount % 50 == 0){ // update every 50 times
		switch(channelCount){
			case 0:
				vA = rtA - prA;	 // get elapsed counts. Should not overflow/underflow.
				// pidA.update();
				csA = analogRead(MT_A_CS);
				MOVEMOTOR(pA, MT_A_DIR, MT_A_BRK, MT_A_PWM);				
				prA = rtA;	   // cleanup
				channelCount++;
				break;
			case 1:
				vB = rtB - prB;	 // get elapsed counts. Should not overflow/underflow.
				// pidB.update();
				csB = analogRead(MT_B_CS);
				MOVEMOTOR(pB, MT_B_DIR, MT_B_BRK, MT_B_PWM);
				prB = rtB;	   // cleanup
				channelCount++;
				break;
			case 2:
				vC = rtC - prC;	 // get elapsed counts. Should not overflow/underflow.
				// pidC.update();
				// pC += pidOutC;
				csC = analogRead(MT_C_CS);
				MOVEMOTOR(pC, MT_C_DIR, MT_C_BRK, MT_C_PWM);
				prC = rtC;	   // cleanup
				channelCount++;
				break;
			case 3:
				vD = rtD - prD;	 // get elapsed counts. Should not overflow/underflow.
				// pidD.update();
				csD = analogRead(MT_D_CS);
				csE = analogRead(MT_E_CS);
				MOVEMOTOR(pD, MT_D_DIR, MT_D_BRK, MT_D_PWM);
				MOVEMOTOR(pE, MT_E_DIR, MT_E_BRK, MT_E_PWM);
				prD = rtD;	   // cleanup

				ledBlink();
				channelCount = 0;
				break;
		}
		if (encoderUpdateCount == 50 * 500){
			encoderUpdateCount = 0;
			if (spiRequestCount == spiRequestCount_prev){
				ledBlinkTime = 30;
				// make sure robot doesn't go crazy when master crashes
				pA = 0;
				pB = 0;
				pC = 0;
				pD = 0;
				pE = 0;
			}
			else{
				ledBlinkTime = 500;
			}
			spiRequestCount_prev = spiRequestCount;
		}
	}
	encoderUpdateCount++;
	// interrupts();
}