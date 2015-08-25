/*
 * slave2.cpp - slave2 program for Team Pi
 * 
 *
 * by Brian Chen
 * Originally authored in November 2014
 * (C) Team PI 2015
 */


#include <WProgram.h>
#include <t3spi.h>
#include <TSOPS.h>
#include <piCommon.h>
#include <DebugUtils.h>


/**********************************************************/
/*					   General						  */
/**********************************************************/

#define DEBUG_SERIAL
#define LED 13
uint8_t status = 0;	 // status of program/slave

/**********************************************************/
/*				  Battery monitoring					*/
/**********************************************************/
#define VBAT_LV_PIN A11
#define VBAT_HV_PIN A10

volatile uint8_t vbatLV, vbatHV;
/**********************************************************/
/*						 SPI							*/
/**********************************************************/

T3SPI SPI; // create SPI object
volatile uint8_t dataOut[2];
volatile uint8_t command = 255;
volatile int16_t tsopAngleVol;
volatile uint8_t tsopStrengthVol;

// blink
elapsedMillis ledElapsedTime;

bool ledState = true;
uint32_t ledBlinkTime = 500;

volatile uint32_t spiRequestCount = 0;
uint32_t spiRequestCount_prev = 0;

/**********************************************************/
/*						TSOPS						   */
/**********************************************************/
/* 40kHz Anything lower than 25us is overkill. For some reason though
   it seems like it actually updates every 20uS when I read using
   elapsedMicros */
   // wait 
#define ISR_INTERVAL 50
//#define ISR_INTERVAL 2500

#define TSOP_RESOLUTION 255

bool tsopsOn = true;


IntervalTimer TSOP_ISR_Timer;
volatile uint32_t TSOP_ISR_Count = 0;

TSOPS tsops; // TSOPS object

/* though tsops already has a member for the data, it's continuously f
   updated and incomplete this variable array stores the most recent 
   tsop data that has been complete (after 255 interrupts) */
uint8_t tsopData[TSOP_COUNT]; 

inline void initBatReadings();
inline void readBat();
inline void initSPI();
inline void ledBlink();
void TSOP_ISR();

extern "C" int main(){
	noInterrupts();

	CORE_PIN33_CONFIG = 0;  // completely disables the pin
	
	pinMode(13, OUTPUT);

	digitalWrite(13, LOW);
	Serial.begin(115200); // nice number but it doesn't actually matter for the Teensy 3.x
	digitalWrite(13, LOW);

	tsops.begin();	 // initialise TSOPS
	tsops.angle = -99; // for debugging purposes, initially set angle as this
	tsops.strength = 150;

	digitalWrite(13, LOW);
	initBatReadings(); // initialise battery readings
	digitalWrite(13, LOW);

	initSPI();		 // initialise SPI slave
	digitalWrite(13, HIGH);
	/* set TSOP ISR priority 0 to 255, 0 being the highest (127 default)
	   we don't want the interrupt to interfer with SPI too much. However */
	TSOP_ISR_Timer.priority(127);
	digitalWrite(13, LOW);
	// attach TSOP ISR
	TSOP_ISR_Timer.begin(TSOP_ISR, ISR_INTERVAL);
	digitalWrite(13, HIGH);
	interrupts();	  // enable interrupts

	while(1){	
	}

	return 0;		  // it'll never reach here
}

// Initialise battery readings
inline void initBatReadings(){
	pinMode(VBAT_LV_PIN, INPUT);
	pinMode(VBAT_HV_PIN, INPUT);
	analogReference(EXTERNAL);  // IMPORTANT!
	analogReadResolution(8);	// Don't need more than 8 bits of resolution (they're just batteries)
	analogReadAveraging(32);	// highest averaging as we don't need fast sampling
}

// Simple reading of batteries
inline void readBat(){
	vbatLV = analogRead(VBAT_LV_PIN);
	vbatHV = analogRead(VBAT_HV_PIN);
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

// Initialise SPI as slave
inline void initSPI(){
	SPI.begin_SLAVE(ALT_SCK, MOSI, MISO, CS0);
	SPI.setCTAR_SLAVE(8, SPI_MODE0);
	NVIC_SET_PRIORITY(IRQ_SPI0, 127); // set priority
	// Enable the SPI0 Interrupt
	NVIC_ENABLE_IRQ(IRQ_SPI0);
}

/* Interrupt Service Routine to handle incoming data
   For best performance, direct access to hardware SPI registers are used (instead of via lib) */
void spi0_isr(){
	command = SPI0_POPR;
	switch(command){
		//case 255: command = c; SPI0_PUSHR_SLAVE = 0; break;
		case 0: SPI0_PUSHR_SLAVE = 0; break;
		// case SLAVE2_COMMANDS::SLAVE2_CHECK_STATUS: SPI0_PUSHR_SLAVE = status;				 break;
		// case SLAVE2_COMMANDS::TSOP_DATA0:		   SPI0_PUSHR_SLAVE = tsopData[0];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA1:		   SPI0_PUSHR_SLAVE = tsopData[1];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA2:		   SPI0_PUSHR_SLAVE = tsopData[2];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA3:		   SPI0_PUSHR_SLAVE = tsopData[3];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA4:		   SPI0_PUSHR_SLAVE = tsopData[4];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA5:		   SPI0_PUSHR_SLAVE = tsopData[5];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA6:		   SPI0_PUSHR_SLAVE = tsopData[6];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA7:		   SPI0_PUSHR_SLAVE = tsopData[7];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA8:		   SPI0_PUSHR_SLAVE = tsopData[8];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA9:		   SPI0_PUSHR_SLAVE = tsopData[9];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA10:		   SPI0_PUSHR_SLAVE = tsopData[10];		     break;	 
		// case SLAVE2_COMMANDS::TSOP_DATA11:		   SPI0_PUSHR_SLAVE = tsopData[11];		     break;
		// case SLAVE2_COMMANDS::TSOP_DATA12:		   SPI0_PUSHR_SLAVE = tsopData[12];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA13:		   SPI0_PUSHR_SLAVE = tsopData[13];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA14:		   SPI0_PUSHR_SLAVE = tsopData[14];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA15:		   SPI0_PUSHR_SLAVE = tsopData[15];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA16:		   SPI0_PUSHR_SLAVE = tsopData[16];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA17:		   SPI0_PUSHR_SLAVE = tsopData[17];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA18:		   SPI0_PUSHR_SLAVE = tsopData[18];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA19:		   SPI0_PUSHR_SLAVE = tsopData[19];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA20:		   SPI0_PUSHR_SLAVE = tsopData[20];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA21:		   SPI0_PUSHR_SLAVE = tsopData[21];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA22:		   SPI0_PUSHR_SLAVE = tsopData[22];		     break; 
		// case SLAVE2_COMMANDS::TSOP_DATA23:		   SPI0_PUSHR_SLAVE = tsopData[23];		     break; 
		// case SLAVE2_COMMANDS::VBAT_REF_LV:		   SPI0_PUSHR_SLAVE = vbatLV;				 break;
		// case SLAVE2_COMMANDS::VBAT_REF_HV:		   SPI0_PUSHR_SLAVE = vbatHV;				 break;
		case SLAVE2_COMMANDS::TSOP_ANGLE_HIGH:	   
			dataOut[0] = highByte(tsopAngleVol); 
			dataOut[1] = lowByte(tsopAngleVol); 
			SPI0_PUSHR_SLAVE = dataOut[0];
			break;
		case SLAVE2_COMMANDS::TSOP_ANGLE_LOW:	   SPI0_PUSHR_SLAVE = dataOut[1];  break;
		case SLAVE2_COMMANDS::TSOP_ANGLE_BYTE:	   SPI0_PUSHR_SLAVE = tsops.angleByte;	 break;
		case SLAVE2_COMMANDS::TSOP_STRENGTH:	   SPI0_PUSHR_SLAVE = tsopStrengthVol; 	 break;
		default:  SPI0_PUSHR_SLAVE = 0;	break;
	}
	spiRequestCount++;
	SPI0_SR |= SPI_SR_RFDF;	
}

// Routine to read tsops (as well as batteries)
void TSOP_ISR(){
	TSOP_ISR_Count++;	
	if (tsopsOn){
		tsops.read();
	}
	else{
#ifdef DEBUG_SERIAL
		Serial.print(vbatHV);
		Serial.print('\t');
		Serial.print(vbatLV);
		Serial.print('\t');
		Serial.print(tsops.averageStrength);
		Serial.print('\t');
		Serial.println(tsops.angle);
#endif
	}
	// else, we're in a stage where we're unlocking the tsops.

	if (TSOP_ISR_Count % TSOP_RESOLUTION == 0){
		// TSOP_ISR_Count = 0;

		// if tsops aren't on turn them on
		if (!tsopsOn){
			tsops.on();
			tsopsOn = true;
		}
		else{
			tsops.finishRead(); // call this every time you've finished reading

			tsops.filterData();
			tsops.getStrength();
			tsops.getAngle();
			tsopAngleVol = tsops.angle;
			tsopStrengthVol = tsops.averageStrength;
			// tsopAngleVol = -99;
			// tsopStrengthVol = 150;
		}
		// unlock tsops and read batteries. Must be a multiple of RESOLUTION!
		if (TSOP_ISR_Count == TSOP_RESOLUTION * 50){
			// 25500 * 25us = 0.6375s
			tsops.off();
			tsopsOn = false; // this tells the program to wait before turning tsops back on
			readBat();	// don't need to read batteries that often
			TSOP_ISR_Count = 0;
			if (spiRequestCount == spiRequestCount_prev){
				ledBlinkTime = 30;
			}
			else{
				ledBlinkTime = 500;
			}
			spiRequestCount_prev = spiRequestCount;
		}  
	}	
	else{
		ledBlink();
	}
}