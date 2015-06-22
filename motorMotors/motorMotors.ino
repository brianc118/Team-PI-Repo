#include <i2c_t3.h>


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


void MOVEMOTOR(int input, uint8_t d, uint8_t b, uint8_t pwm)
{	
	if (input < 0){					
		digitalWriteFast(d, HIGH);	
		digitalWriteFast(b, LOW);	
		analogWrite(pwm, abs(input));  
	} else if(input > 0){			   
		digitalWriteFast(d, LOW);	   
		digitalWriteFast(b,LOW);		
		analogWrite(pwm,abs(input));	
	} else{							 
		digitalWriteFast(b,HIGH);		   
		analogWrite(pwm,abs(input));
	}
} 


void setup() {
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

	// analogWriteFrequency(MT_A_PWM, PWM_FREQ);
	// analogWriteFrequency(MT_B_PWM, PWM_FREQ);
	// analogWriteFrequency(MT_C_PWM, PWM_FREQ);
	// analogWriteFrequency(MT_D_PWM, PWM_FREQ);
	// analogWriteFrequency(MT_E_PWM, PWM_FREQ);
}

void loop() {
  MOVEMOTOR(50, MT_A_DIR, MT_A_BRK, MT_A_PWM);
  // MOVEMOTOR(50, MT_B_DIR, MT_B_BRK, MT_B_PWM);
  // MOVEMOTOR(50, MT_C_DIR, MT_C_BRK, MT_C_PWM);
  // MOVEMOTOR(50, MT_D_DIR, MT_D_BRK, MT_D_PWM);
  // MOVEMOTOR(50, MT_A_DIR, MT_A_BRK, MT_A_PWM);
}
