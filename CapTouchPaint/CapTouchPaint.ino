/***************************************************
Capacitive touchscreen paint test.

IlI9341 lcd connected interfaced using 8 bit bidirectional
parallel.
FT6206 touch driver interfaced with i2c (max 400mHz)

Uses a modified version of Adafruit_TFTLCD and Adafruit_FT6206 libraries
optimised for the ILI9341 and t3.1.

Native parallel io with the t3.1 is not implement due to
pin limitations, hence LCD performance is poor compared to
the optimised SPI library for the ILI9341. However, performance is still
quite good.

Based on original CapTouchPaint example from Adafruit Industries

by Brian Chen
(C) Team Pi 2014

 ****************************************************/

#include <SPI.h>
#include "ILI9341_t3.h"
#include <i2c_t3.h>      
#include <Adafruit_FT6206.h>

#define ANALOG_RES 12   // 12 bit resolution
#define MAX_BRIGHTNESS (2 ^ ANALOG_RES - 1)
#define BRIGHTNESS 100   // as percentage

#define LITE   A9

#define IRQ    16   // active high interrupt for touch (low when touched)
// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ctp = Adafruit_FT6206();

#define TFT_DC 22
#define TFT_CS 20

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, 255, 11, 14, 12);

// Size of the color selection boxes and the paintbrush size
#define BOXSIZE 40
#define PENRADIUS 3
int oldcolor, currentcolor;

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

void touchDetected(){
  // Retrieve a point  
  POINT p = ctp.getPoint();

  // flip it around to match the screen.
  // p.x = map(p.x, 0, 240, 240, 0);
  // p.y = map(p.y, 0, 320, 320, 0);

  // Print out the remapped (rotated) coordinates
  Serial.print("("); Serial.print(p.x);
  Serial.print(", "); Serial.print(p.y);
  Serial.println(")");
  
  if (p.y < BOXSIZE) {
     oldcolor = currentcolor;

     if (p.x < BOXSIZE) { 
       currentcolor = RED; 
       tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
     } else if (p.x < BOXSIZE*2) {
       currentcolor = YELLOW;
       tft.drawRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, WHITE);
     } else if (p.x < BOXSIZE*3) {
       currentcolor = GREEN;
       tft.drawRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, WHITE);
     } else if (p.x < BOXSIZE*4) {
       currentcolor = CYAN;
       tft.drawRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, WHITE);
     } else if (p.x < BOXSIZE*5) {
       currentcolor = BLUE;
       tft.drawRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, WHITE);
     } else if (p.x <= BOXSIZE*6) {
       currentcolor = MAGENTA;
       tft.drawRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, WHITE);
     }

     if (oldcolor != currentcolor) {
        if (oldcolor == RED) 
          tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
        if (oldcolor == YELLOW) 
          tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
        if (oldcolor == GREEN) 
          tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, GREEN);
        if (oldcolor == CYAN) 
          tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, CYAN);
        if (oldcolor == BLUE) 
          tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, BLUE);
        if (oldcolor == MAGENTA) 
          tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, MAGENTA);
     }
  }
  if (((p.y-PENRADIUS) > BOXSIZE) && ((p.y+PENRADIUS) < tft.height())) {
    tft.fillCircle(p.x, p.y, PENRADIUS, currentcolor);
  }
}

void setup() { 
  Wire.begin(I2C_MASTER, 0, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_800);
  Serial.begin(115200);
  Serial.println(F("Cap Touch Paint!"));

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(BLACK);
  tft.setTextColor(GREEN);
  tft.setTextSize(3);

  if (!ctp.begin(30)) {  // pass in 'sensitivity' coefficient
    Serial.println("Couldn't start FT6206 touchscreen controller");
    tft.println("Couldn't start FT6206 touchscreen controller");
    while (1);
  }

  Serial.println("Capacitive touchscreen started");
  
  // make the color selection boxes
  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, RED);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, YELLOW);
  tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, GREEN);
  tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, CYAN);
  tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, BLUE);
  tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, MAGENTA);
 
  // select the current color 'red'
  tft.drawRect(0, 0, BOXSIZE, BOXSIZE, WHITE);
  currentcolor = RED;

  pinMode(IRQ, INPUT);
  attachInterrupt(IRQ, touchDetected, LOW);

  pinMode(LITE, OUTPUT);
  analogWriteResolution(12);
  delay(1);
  analogWrite(LITE, (int)(BRIGHTNESS * MAX_BRIGHTNESS / 100));
}

void loop() {
  analogWrite(LITE, (int)(1000));
  //Serial.println(digitalRead(IRQ));
  // Wait for a touch
  // if (digitalRead(IRQ) == HIGH) {
  //   return;
  // }
  touchDetected();  
}