

#include <Adafruit_GFX.h>   
#include <Adafruit_TFTLCD.h> 

#define LCD_CS 31
#define LCD_CD 28
#define LCD_WR 22
#define LCD_RD 30 //not used

#define LCD_RESET 29 // Can alternately just connect to Arduino's reset pin


// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  // while(Serial.available() <= 0){
  //   Serial.println("hi");
  // }
  Serial.println(F("TFT LCD test"));
  Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
  tft.reset();
  Serial.println("TFT has been reset");
  uint16_t identifier = 0x9341;
  
  Serial.println(F("Benchmarking speed of outputting numbers as data"));
  
  tft.begin(identifier);
  tft.setRotation(0);
  tft.setTextColor(WHITE);  tft.setTextSize(10);
  while(true){
      tft.fillScreen(BLACK);

  }
}

void loop() {
  digitalWrite(13, HIGH);
  delay(50);
  unsigned long start = micros();

  for (int i = 10000; i < 10000 + 9; i++){
    tft.fillScreen(BLACK);
    tft.setCursor(0, 0);
    tft.println(i);
    Serial.println(i);
  }
  unsigned long dt = micros();
  digitalWrite(13, LOW);
  dt -= start;
  float fps = 999000000 / dt;
  tft.fillScreen(BLACK);
  tft.setCursor(0, 0);
  tft.println("fps");
  tft.println(fps);
  delay(50);
}