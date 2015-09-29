/* code by fdavies

   This example code is in the public domain.
   
   The goal is to have a complete 3D printer driver (gcode => motion)
   that uses optical encoders and brushed DC motors.
*/

// definition of how the pins are set up

const int xQ1Pin = 7;  // pin for x axis quadrature encoder    J1 pin 4
const int xQ2Pin = 8;  // pin for x axis quadrature encoder    J1 pin 3
//(GPIOD_PDIR & 0x000C) >> 2    expression for reading the pins directly 
const int xHPin = 9;   // pin for x axis home signal               // not implemented yet  J1 pin 2
const int xM1Pin = 3;  // pin for x axis motor control M1    goes to IN1 pin of seeedstudio motor shield v2.0  J1 pin 6
const int xM2Pin = 4;  // pin for x axis motor control M2    goes to IN2 pin of seeedstudio motor shield v2.0  J1 pin 9

const int yQ1Pin = 10;  // pin for y axis quadrature encoder   J2 pin 4
const int yQ2Pin = 11;  // pin for y axis quadrature encoder   J2 pin 3
//((GPIOC_PDIR & 0x0010) >> 4) | ((GPIOC_PDIR & 0x0040) >> 5)   expression for reading the pins directly 
const int yHPin = 12;   // pin for y axis home signal            J2 pin 2
const int yM1Pin = 5;   // pin for y axis motor control M1        goes to IN3 pin of seeedstudio motor shield v2.0  J2 pin 6
const int yM2Pin = 6;   // pin for y axis motor control M2        goes to IN4 pin of seeedstudio motor shield v2.0  J2 pin 9

const int zQ1Pin = 14;  // pin for z axis quadrature channel 1       J3 pin 4
const int zQ2Pin = 15;  // pin for z axis quadrature channel 2       J3 pin 3
//((GPIOD_PDIR & 0x0002) >> 1) | ((GPIOC_PDIR & 0x0001) << 1)   expression for reading the pins directly 
const int zHPin = 24;   // pin for z axis home signal, may change 
const int zM1Pin = 20;  // pin for z axis motor control M1       will go to another seeedstudio motor shield when I get it hitched up
const int zM2Pin = 21;  // pin for z axis motor control M2        

const int eQ1Pin = 16;  // pin for e axis quadrature channel 1    J4 pin 4
const int eQ2Pin = 17;  // pin for e axis quadrature channel 2    J4 pin 4
//((GPIOB_PDIR & 0x0003) >> 0)   expression for reading the pins directly 
const int eHPin = 25;   // pin for e axis home signal  may change  25 B 19
const int eM1Pin = 22;  // pin for e axis motor control M1         
const int eM2Pin = 23;  // pin for e axis motor control M2         

const int motorEnable = 2; // pin to enable the motor drivers (all)  goes to EA and EB of seeedstudio motor shield v2.0 
               
const int outMin = 0; // minimum output level (PWM)
const int outMid = 128;  // middle output level (PWM)
const int outMax = 256;  // maximum output level (PWM)


// Teensy 3.0 has the LED on pin 13
const int ledPin = 13;
int p=0;
// this is the decode table for the optical encoder
signed long enc_TAB[]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

// the setup() method runs once, when the sketch starts

// Create an IntervalTimer object 
IntervalTimer encoder;

volatile unsigned long encoderState = 0; // use volatile for shared variables

volatile signed long xEncoder = 0; // This is the position based on counting encoder pulses
volatile signed long yEncoder = 0; // This is the position based on counting encoder pulses
volatile signed long zEncoder = 0; // This is the position based on counting encoder pulses
volatile signed long eEncoder = 0; // This is the position based on counting encoder pulses

volatile signed long xSetPoint = 0; // this is where the PID loop will try to move the axis
volatile signed long ySetPoint = 0;
volatile signed long zSetPoint = 0;
volatile signed long eSetPoint = 0;

unsigned int count1 = 0;  // so that the encoder counting is faster than the PID updating
unsigned int channelCount = 0;  // 

signed long xki=0;  // PID integration constant
signed long xkp=1024;  // PID proportional constant
signed long xkd=1024;  // PID derivative constant
signed long xITerm=0;
signed long xLastInput=0;

signed long yki=0;  // PID integration constant
signed long ykp=1024;  // PID proportional constant
signed long ykd=1024;  // PID derivative constant
signed long yITerm=0;
signed long yLastInput=0;

signed long zki=0;  // PID integration constant
signed long zkp=0;  // PID proportional constant
signed long zkd=0;  // PID derivative constant
signed long zITerm=0;
signed long zLastInput=0;

signed long eki=0;  // PID integration constant
signed long ekp=0;  // PID proportional constant
signed long ekd=0;  // PID derivative constant
signed long eITerm=0;
signed long eLastInput=0;

//unsigned long lastTime;

void setup() {
  // initialize pins for the motor control channels
  int k=0;
  
  Serial.begin(38400);  
  Serial.println(" step 1");
    
  pinMode(A14,OUTPUT);  
  //analogWriteResolution(12);
    
  pinMode(xQ1Pin, INPUT);  
  pinMode(xQ2Pin, INPUT);  
  pinMode(xHPin, INPUT);  
  analogWriteFrequency(xM1Pin, 20000); // change PWM frequency out of audible range
  pinMode(xM1Pin, OUTPUT);  // default frequency and resolution for now
  pinMode(xM2Pin, OUTPUT);  // default frequency and resolution for now  
  
  pinMode(yQ1Pin, INPUT);  
  pinMode(yQ2Pin, INPUT);  
  pinMode(yHPin, INPUT);  
  analogWriteFrequency(yM1Pin, 20000); // change PWM frequency
  pinMode(yM1Pin, OUTPUT);  // default frequency and resolution for now
  pinMode(yM2Pin, OUTPUT);  // default frequency and resolution for now    

  pinMode(zQ1Pin, INPUT);  
  pinMode(zQ2Pin, INPUT);  
  pinMode(zHPin, INPUT);  
  pinMode(zM1Pin, OUTPUT);  // default frequency and resolution for now
  pinMode(zM2Pin, OUTPUT);  // default frequency and resolution for now    

  pinMode(eQ1Pin, INPUT);  
  pinMode(eQ2Pin, INPUT);  
  pinMode(eHPin, INPUT);  
  pinMode(eM1Pin, OUTPUT);  // default frequency and resolution for now 488.28 Hz
  pinMode(eM2Pin, OUTPUT);  // default frequency and resolution for now    
  
  pinMode(ledPin, OUTPUT);
  pinMode(motorEnable,OUTPUT);

  digitalWrite(motorEnable,HIGH);  // enable the motor drivers  



  //https://www.pjrc.com/teensy/td_timing_IntervalTimer.html
  

  
  /* move all the motors weakly negative for 2 seconds to home them.  Do this better later */
  k=4;
  analogWrite(xM2Pin,(outMin*(8+k)+outMax*(8-k))/16);
  analogWrite(xM1Pin,(outMin*(8-k)+outMax*(8+k))/16);    
  analogWrite(yM2Pin,(outMin*(8+k)+outMax*(8-k))/16);
  analogWrite(yM1Pin,(outMin*(8-k)+outMax*(8+k))/16); 
  delay(2000);
  Serial.println(" step 3");    
  encoder.begin(encoderUpdate, 20);  // blinkLED to run every 0.02 millisecond 50 KHz
  
}

// functions called by IntervalTimer should be short, run as quickly as
// possible, and should avoid calling other functions if possible.
void encoderUpdate(void) {
  unsigned long nowState = 0 ;
  unsigned long encIndex =0;
  signed long error=0; // for PID loops
  signed long dInput=0; // for PID loops
    signed long Output=0; // for PID loops
   
  digitalWriteFast(ledPin,1); // for measuring interrupt duration with an oscilloscope
  
  /* nowState = (digitalRead(xQ2Pin)<<1) | digitalRead(xQ1Pin)      | (digitalRead(yQ2Pin)<<5)  | (digitalRead(yQ1Pin)<<4) | \
             (digitalRead(zQ2Pin)<<9) | (digitalRead(zQ1Pin)<<8) | (digitalRead(eQ2Pin)<<13) | (digitalRead(eQ1Pin)<<12);  // 2.5 microseconds */

  nowState = (digitalReadFast(xQ2Pin)<<1) | digitalReadFast(xQ1Pin)      | (digitalReadFast(yQ2Pin)<<5)  | (digitalReadFast(yQ1Pin)<<4) | \
             (digitalReadFast(zQ2Pin)<<9) | (digitalReadFast(zQ1Pin)<<8) | (digitalReadFast(eQ2Pin)<<13) | (digitalReadFast(eQ1Pin)<<12); 
             
  /*nowState = (GPIOD_PDIR & 0x000C) >> 2  | \
             ((((GPIOC_PDIR & 0x0010) >> 4) | ((GPIOC_PDIR & 0x0040) >> 5) ) << 4) | \
             ((((GPIOD_PDIR & 0x0002) >> 1) | ((GPIOC_PDIR & 0x0001) << 1)) <<8 ) |  \
             (( (GPIOB_PDIR & 0x0003) >> 0) <<12) ;         */
             
  encIndex=nowState | (encoderState << 2);
             
  xEncoder = xEncoder + enc_TAB[ (encIndex >> 0) & 0x0f];  // these 4 lines take about 0.7 microseconds, maybe can be optimized
  yEncoder = yEncoder + enc_TAB[ (encIndex >> 4) & 0x0f];
  zEncoder = zEncoder + enc_TAB[ (encIndex >> 8) & 0x0f];
  eEncoder = eEncoder + enc_TAB[ (encIndex >> 12) & 0x0f];  

  encoderState = nowState; 

  count1++;
  if (count1 >= 50) {  // aiming for 1 khz update rate
    count1 = 0; // restart the count
    channelCount++;
    if (channelCount >= 4) {
     channelCount = 0;
    }
    switch (channelCount) {  // actually each channel has its PID updated at 250Hz, which seems faste enough so far.
    case 0: // x channel
      {
      /***********************************************************************
      **
      **   PID stuff closely based on
      **   // brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
      **
      ************************************************************************/
      digitalWriteFast(ledPin,1);  // as it stands, this takes about 3 microseconds
      /*Compute all the working error variables*/
      error = xSetPoint - xEncoder;
      xITerm+= (xki * error*1024);  // scaling for fixed point 
      
      if(xITerm > outMax*1024) xITerm= outMax*1024;
      else if(xITerm < outMin*1024) xITerm= outMin*1024;
 
      Output = (xkp * error + xITerm- xkd * dInput)/1024;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
      
      analogWrite(xM2Pin,Output);
      analogWrite(xM1Pin,outMax-Output);     
 
      xLastInput = xEncoder;  
      
      digitalWriteFast(ledPin,0);
      }
      break;
    case 1: // y channel
      error = ySetPoint - yEncoder;
      yITerm+= (yki * error*1024);  // scaling for fixed point
      
      if(yITerm > outMax*1024) yITerm= outMax*1024;
      else if(yITerm < outMin*1024) yITerm= outMin*1024;
 
      Output = (ykp * error + yITerm- ykd * dInput)/1024;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
      
      analogWrite(yM2Pin,Output);
      analogWrite(yM1Pin,outMax-Output);     
 
      yLastInput = yEncoder;  
          
      break;
    case 2: // z channel
      break;
    case 3: // e channel
      break;      
    }
    //Calculate the distance between the setpoint and encoder values
    // this is my first start at getting a way to tune the two channels so they work together
   analogWrite(A14,(xEncoder+yEncoder-2100)/16+128);
    // xSetPoint-xEncoder)*(xSetPoint-xEncoder) +  (ySetPoint-yEncoder)*(ySetPoint-yEncoder)
  }
  
  digitalWriteFast(ledPin,0); // for measuring interrupt duration with an oscilloscope

}

// the loop() methor runs over and over again,
// as long as the board has power

void loop() {
  int i;
  signed long copy_xEncoder= 0;
  signed long copy_yEncoder= 0;
  signed long copy_zEncoder= 0;
  signed long copy_eEncoder= 0;

  
    cli(); // disable interrupt
    copy_xEncoder = xEncoder;
    copy_yEncoder = yEncoder;
    copy_zEncoder = zEncoder;
    copy_eEncoder = eEncoder;
    sei(); // reenable the interrupt 
    
    Serial.print(copy_xEncoder,DEC);
    Serial.print(" ");
    Serial.print(copy_yEncoder,DEC);
    Serial.print(" ");
    Serial.print(copy_zEncoder,HEX);
    Serial.print(" ");
    Serial.print(copy_eEncoder,HEX);
    Serial.println(" ");
  
  xSetPoint = 1000;  // this is how the PID control loop is told to move the axis.
  ySetPoint = 3000;
  delay(100); // 

  for (i=0;i<2000;i++)  {  // test profile
    delay(2);
    xSetPoint = 3000 - i;
    ySetPoint = 1000 + i;
  }
  
  delay(100); // 
  xSetPoint = 1000;    
  ySetPoint = 3000;

}
// see.stanford.edu/materials/aiircs223a/handout6_Trajectory.pdf
// developer.mbed.org/cookbook/PID
// brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
// www.embeddedrelated.com/showarticle/121.ph