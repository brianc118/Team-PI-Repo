/*
 * slave2.cpp - slave2 program for Team Pi
 * 
 *
 * by Brian Chen
 * (C) Team Pi 2014
 */


#include <WProgram.h>
#include <t3spi.h>
#include <TSOPS.h>
#include <piCommon.h>
#include <DebugUtils.h>


/**********************************************************/
/*					   General						  */
/**********************************************************/
#define LED 13
uint8_t status = 0;	 // status of program/slave



T3SPI SPI; // create SPI object
volatile uint8_t dataOut, dataIn; // SPI input/output bytes
volatile uint8_t command = 255;




int main(){
	noInterrupts();

	CORE_PIN33_CONFIG = 0;  // completely disables the pin
	
	pinMode(13, OUTPUT);
	Serial.begin(115200); 

	SPI.begin_SLAVE(ALT_SCK, MOSI, MISO, CS0);
	SPI.setCTAR_SLAVE(8, SPI_MODE0);
	NVIC_SET_PRIORITY(IRQ_SPI0, 127); // set priority
	// Enable the SPI0 Interrupt
	NVIC_ENABLE_IRQ(IRQ_SPI0);

	interrupts();


	while(1){	
	}

	return 0;		  // it'll never reach here
}

/* Interrupt Service Routine to handle incoming data
   For best performance, direct access to hardware SPI registers are used (instead of via lib) */
void spi0_isr(){
	command = SPI0_POPR;
	switch(command){
		case 0: SPI0_PUSHR_SLAVE = 0; break;
		case SLAVE2_COMMANDS::TSOP_ANGLE_BYTE:	   SPI0_PUSHR_SLAVE = 55;	 break;
		case SLAVE2_COMMANDS::TSOP_STRENGTH:	   SPI0_PUSHR_SLAVE = 155; 	 break;
		default:  SPI0_PUSHR_SLAVE = 0;	break;
	}

	SPI0_SR |= SPI_SR_RFDF;
}