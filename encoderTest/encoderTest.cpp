/*
 * Stripped down version of original t3 spi slave program
 * Should work but not yet tested for 4 motor decoders
 *
 * X corresponds to A, Y to B, Z to C etc.
 *
 * Comment out hardwaredecode to disable hardware decoding and use
 * pure software decoding on all 4 motors
 * Uncomment this to enable hardware decoding on the X and Y channels
 */

#include <WProgram.h>
#include <QuadDecode_def.h> // quad decoder library (hardware)

#define LED 13

//#define HARDWAREDECODE // comment out to disable all hardware decoding

#define SOFTWARE_UPDATE_INTERVAL 18 // time between each update in us
#define ENCXA       3  //|
#define ENCXB       4  //| Use built in hardware quadrature interface
#define ENCYA       32 //|
#define ENCYB       25 //|
#define ENCZA       19
#define ENCZB       18
#define ENCEA       21
#define ENCEB       22

#ifdef HARDWAREDECODE
QuadDecode<1> xPosn;	// Template using FTM1
QuadDecode<2> yPosn;	// Template using FTM2
#endif

volatile int32_t rtA = 0, rtB = 0, rtC = 0, rtD = 0;	// Realtime values
		 int32_t prA = 0, prB = 0, prC = 0, prD = 0;	// Previous values

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

volatile int32_t channelCount = 0, encoderUpdateCount = 0;

void encoderUpdate();

int main(void){
	noInterrupts();
	Serial.begin(115200);

	pinMode(LED, OUTPUT);


#ifdef HARDWAREDECODE
	xPosn.start();	    // Start Quad Decode position count
	yPosn.start();	    // Start Quad Decode position count
#endif

	encoderUpdater.priority(0);
	encoderUpdater.begin(encoderUpdate, SOFTWARE_UPDATE_INTERVAL);

	interrupts();
	// main loop
	while(1){
		// the following is not tested
		Serial.print(rtA); Serial.print('\t');
		Serial.print(rtB); Serial.print('\t');
		Serial.print(rtC); Serial.print('\t');
		Serial.print(rtD); Serial.println();
	}
}

// this routine is called via an interrupt to update the encoders
// from my experience, doing anything more than one pid calculation will be > 20uS
// this means you should really only process for 1 motor at a time.
void encoderUpdate(){
	noInterrupts(); // not sure if necessary
	digitalWriteFast(LED, LOW);

#ifdef HARDWAREDECODE
	// software decode Z and E channels
	encState = (digitalReadFast(ENCZB)<<9) | (digitalReadFast(ENCZA)<<8) | (digitalReadFast(ENCEB)<<13) | (digitalReadFast(ENCEA)<<12);
	// hardware decode X and Y channels
	rtA = xPosn.calcPosn();
	rtB = yPosn.calcPosn();
#else
	encState = (digitalReadFast(ENCXB)<<1) | (digitalReadFast(ENCXA))    | (digitalReadFast(ENCYB)<<5)  | (digitalReadFast(ENCYA)<<4) 
			 | (digitalReadFast(ENCZB)<<9) | (digitalReadFast(ENCZA)<<8) | (digitalReadFast(ENCEB)<<13) | (digitalReadFast(ENCEA)<<12);
#endif

    encIndex = encState | (lEncState << 2);

    /* byte   4    3    2    1
              lB   lA   B    A */


    // if anything the following 4 lines are the best to optimise
#ifndef HARDWAREDECODE
    rtA += enc_TAB[ (encIndex >> 0)  & 0x0f]; // we are only using 4 bytes per encoder
	rtB += enc_TAB[ (encIndex >> 4)  & 0x0f];
#endif
	rtC += enc_TAB[ (encIndex >> 8)  & 0x0f];
	rtD += enc_TAB[ (encIndex >> 12) & 0x0f];

	lEncState = encState;

	// encoder decoding stuff done. Begin PID
	if (encoderUpdateCount == 50){ // update every 50 times
		encoderUpdateCount = 0;
		channelCount++;
		switch(channelCount){
			case 0:
				vA = rtA - prA;	 // get elapsed counts. Should not overflow/underflow.
				// pid here
				prA = rtA;       // cleanup
				break;
			case 1:
				vB = rtB - prB;	 // get elapsed counts. Should not overflow/underflow.
				// pid here
				prB = rtB;       // cleanup
				break;
			case 2:
				vC = rtC - prC;	 // get elapsed counts. Should not overflow/underflow.
				// pid here
				prB = rtC;       // cleanup
				break;
			case 3:
				vD = rtD - prD;	 // get elapsed counts. Should not overflow/underflow.
				// pid here
				prD = rtD;       // cleanup
				channelCount = 0;
				break;
		}
	}
	encoderUpdateCount++;
	interrupts(); // not sure if necessary
}