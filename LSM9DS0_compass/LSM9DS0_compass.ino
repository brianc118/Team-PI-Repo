/*
 * LSM9DS0 compass
 * DOUT    --> MOSI  --> 12 (default)
 * DIN     --> MISO  --> SDOG  --> SDOXM --> 13 (default)
 * CSXM    --> 17
 * CSG     --> 14
 * INT2XM  --> 8
 * INT1XM  --> 9
 * DRDG    --> 10
 * Based on original LSM9DS_AHRS example from Sparkfun

 * by Brian Chen
 * (C) Team Pi 2014
 */

#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <i2c_t3.h>
#include <SFE_LSM9DS0.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_TFTLCD.h>
// #include <Adafruit_FT6206.h>

#define SWAP(a, b) {int c = a; a = b; b = c; }

/**********************************************************/
/*                        LSM9DS0                         */
/**********************************************************/
// Magnetic Declination/Inclination - see
// http://autoquad.org/wiki/wiki/configuring-autoquad-flightcontroller/autoquad-calibrations/calibration-faq/magnetic-declination-and-inclination/
// Data acquired from http://magnetic-declination.com
#define DECLINATION  10.92 // (Newmarket, Brisbane, Australia)
#define INCLINATION  57.37 // (Newmarket, Brisbane, Australia)

// Uncomment when in China
// #define DECLINATION  -5.03 // (Hefei, China)
// #define INCLINATION  48.18 // (Hefei, China)

#define LSM9DS0_CSG  14  // CSG connected to Arduino pin 9
#define LSM9DS0_CSXM 17 // CSXM connected to Arduino pin 10
LSM9DS0 dof(LSM9DS0_CSG, LSM9DS0_CSXM);

#define INT1XM 9 // INT1XM tells us when accel data is ready
#define INT2XM 8 // INT2XM tells us when mag data is ready
#define DRDYG  10 // DRDYG  tells us when gyro data is ready


#define GyroMeasError PI * (40.0f / 180.0f)     // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)

#define beta sqrt(3.0f / 4.0f) * GyroMeasError  // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float pitch, yaw, roll, heading;
float deltat = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0;    // used to calculate integration interval
uint32_t Now = 0;           // used to calculate integration interval

float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float temperature;

/**********************************************************/
/*                      TFT Display                       */
/**********************************************************/

#define LCD_CS 15   // chip select
#define LCD_CD 20   // command/data
#define LCD_WR 22   // write
#define LCD_RD 21   // read

#define LCD_RESET 23

Adafruit_TFTLCD lcd(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// get any RGB value to 16-bit colour value
#define RGB(r,g,b) \
    (((31*(r+4))/255)<<11) |   \
    (((63*(g+2))/255)<<5)  |   \
    ((31*(b+4))/255)


// 16-bit colour values:
#define BLACK    0x0000
#define TRIFIFTY 0x3186
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

uint16_t standard_colours[] = { BLACK, TRIFIFTY, BLUE, RED, GREEN, CYAN, MAGENTA, YELLOW, WHITE };



/*                        Touch                           */
#define IRQ    16   // active high interrupt for touch (low when touched)
// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 capTouch = Adafruit_FT6206();

/*                          UI                            */
class UI
{
public:
  UI (Adafruit_TFTLCD &t, Adafruit_FT6206 &c, uint8_t touchIntteruptPin, uint8_t orient, 
              uint16_t bColour, uint16_t tColour, uint8_t ts, uint8_t ss, uint8_t ns){
    tft = &t;
    ctp = &c;
    backColour = bColour;
    textColour = tColour;   
    orientation = orient;

    irq = touchIntteruptPin;

    titleSize = ts;
    subTitleSize = ss;
    normalSize = ns;

    if (orientation == 1 || orientation == 3){
      uint16_t temp;
      temp = width;
      width = height;
      height = temp;
    }
  }
  bool begin(){
    tft -> reset();
    tft -> begin(0x9341);

    tft -> setRotation(orientation);
    tft -> fillScreen(backColour);
    tft -> setTextColor(textColour);
    tft -> setTextSize(titleSize);

    pinMode(irq, INPUT);
    if (!capTouch.begin(30)) {  // pass in 'sensitivity' coefficient
      tft -> println("Couldn't start FT6206 touchscreen controller");
      return false;
    }
    capTouch.setRotation(orientation);
  }
  void drawMenu(String text[], uint8_t columns, uint8_t rows){
    drawMenu(text, rows, columns, standard_colours);
  }
  void drawMenu(String text[], uint8_t columns, uint8_t rows, uint16_t colours[]){
    uint8_t i = 0;
    r = rows;
    c = columns;
    boxWidth = (width / c);
    boxHeight = (height / r);

    tft -> setTextSize(titleSize);
    for (uint16_t y = 0; y < boxHeight * r; y += boxHeight){
      for (uint16_t x = 0; x < boxWidth * c; x += boxWidth){        
        tft -> fillRect(x, y, boxWidth, boxHeight, colours[i]);
        // tft -> setCursor(x, y);
        // 6 x 8 pixel text. Text is scalled by textSize
        tft -> setCursor(x + (boxWidth - 6 * titleSize * text[i].length()) / 2, 
          y + (boxHeight - 8 * titleSize) / 2);
        tft -> print(text[i]);
        i++;
      }
    }
  }
  void drawConsole(){
    drawConsole(consoleHeight);
  }
  void drawConsole(uint16_t cH){
    consoleHeight = cH;
    tft -> fillRect(0, height - consoleHeight, width, consoleHeight, backColour);
  }
  void printToConsole(String s){
    tft -> setTextSize(normalSize);
    // check if text will overflow
    uint16_t h = height - consoleHeight + 5 + consoleLine * normalSize * 8;
    if (h + normalSize * 8 > height){
      drawConsole();
      consoleLine = 0;
    }
    tft -> setCursor(5, height - consoleHeight + 5 + consoleLine * normalSize * 8);
    tft -> print(s);
    consoleLine ++;
  }
  // return touched box
  uint8_t checkForTouched(){
    if (digitalReadFast(irq) == HIGH){
      // not touched
      return 255;
    }
    POINT p = ctp -> getPoint();
    // now check which box it's in.
    uint8_t column, row;
    for (uint16_t y = 0; y < r * boxHeight; y += boxHeight){
      if (p.y >= y && p.y <= y + boxHeight){
        // found the row
        row = y / boxHeight;
      }
    }
    for (uint16_t x = 0; x < c * boxWidth; x += boxWidth){
      if (p.x >= x && p.x <= x + boxWidth){
        // found the row
        column = x / boxWidth;
      }
    }
    return (column + c * row);
  }
  void drawWindow(){

  }
private:
  Adafruit_TFTLCD *tft;
  Adafruit_FT6206 *ctp;
  uint16_t width = 240, height = 320; // dimensions of display

  uint16_t backColour, textColour;    
  uint8_t irq;
  uint8_t orientation;
  uint8_t titleSize, subTitleSize, normalSize;

  // menu
  uint16_t  boxWidth, boxHeight;      // dimensions of menu boxes
  uint8_t r, c;

  // console
  uint16_t consoleHeight = 150;       // height of console from bottom
  uint8_t consoleLine = 0;            // line number of console currently
};

UI gui(lcd, capTouch, IRQ, 1, BLACK, WHITE, 3, 2, 1);

/**********************************************************/
/*                       Main Code                        */
/**********************************************************/
uint32_t ii = 0;
void setup()
{
  Serial.begin(9600);
  //while(Serial.available() <= 0){};

  Wire.begin(I2C_MASTER, 0, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
  
 
  // Set up interrupt pins as inputs:
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG,  INPUT);

            
  uint32_t status = dof.begin();

  dof.setAccelScale(dof.A_SCALE_2G);
  dof.setGyroScale(dof.G_SCALE_245DPS);
  dof.setMagScale(dof.M_SCALE_2GS);
  dof.setAccelABW(dof.A_ABW_50);
  dof.setGyroODR(dof.G_ODR_190_BW_125);  
  dof.setMagODR(dof.M_ODR_125); 
  dof.calLSM9DS0(gbias, abias);

  
  //gui.begin();
  delay(1000);
  
}

void loop()
{
  // String menuItems[] = { "TSOPs", "IMU", "Motors", "Test", "Lt Sensors", "Console" };
  // uint16_t colours[] = { RGB(50,50,50), BLACK, BLACK, RGB(50,50,50), RGB(50,50,50), BLACK };  
  // gui.drawMenu(menuItems, 2, 3, colours);

  // uint8_t touched;
  // // lcd.fillScreen(BLACK);
  // do {
  //   touched = gui.checkForTouched();
  // } while (touched == 255);
  // lcd.fillScreen(BLACK);
  // lcd.setCursor(0,0);
  // lcd.println(touched);
  // delay(1000);
  
  
  // switch (touched){
  //   case 0: break;
  //   case 1:
  //     gui.drawConsole(240);
  //     do{
  //       compass(); // print out compass data to display
  //       } while(!capTouch.touched()); // check if display is touched
  //     break;
  //   case 2: break;
  //   case 3: break;
  //   case 4: break;
  //   case 5: break;
  // }
  // delay(500);
  compass();
}

void printHeading(float hx, float hy){
  if (hy > 0){
    heading = 90 - (atan(hx / hy) * (180 / PI));
  }
  else if (hy < 0){
    heading = - (atan(hx / hy) * (180 / PI));
  }
  else{
    if (hx < 0) heading = 180;
    else heading = 0;
  }
  
  // Serial.print("Heading: ");
  // Serial.println(heading, 2);
}

inline void compass(){
  if(digitalReadFast(DRDYG)) {  // When new gyro data is ready
  dof.readGyro();           // Read raw gyro data
    gx = dof.calcGyro(dof.gx) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
    gy = dof.calcGyro(dof.gy) - gbias[1];
    gz = dof.calcGyro(dof.gz) - gbias[2];
  }
  
  if(digitalReadFast(INT1XM)) {  // When new accelerometer data is ready
    dof.readAccel();         // Read raw accelerometer data
    ax = dof.calcAccel(dof.ax) - abias[0];   // Convert to g's, remove accelerometer biases
    ay = dof.calcAccel(dof.ay) - abias[1];
    az = dof.calcAccel(dof.az) - abias[2];
  }
  
  if(digitalReadFast(INT2XM)) {  // When new magnetometer data is ready
    dof.readMag();           // Read raw magnetometer data
    mx = dof.calcMag(dof.mx);     // Convert to Gauss and correct for calibration
    my = dof.calcMag(dof.my);
    mz = dof.calcMag(dof.mz);
    
    dof.readTemp();
    temperature = 21.0 + (float) dof.temperature/8.; // slope is 8 LSB per degree C, just guessing at the intercept
  }

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;
  // Sensors x- and y-axes are aligned but magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);
  //MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);

  // Print the heading and orientation for fun!
  // printHeading(mx, my);

  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  yaw   *= 180.0f / PI; 
  yaw   -= DECLINATION;

  Serial.println(yaw, 2);
  // gui.printToConsole(String(yaw, 2) + "\t" + String(1/deltat, 1) + String(ii));
  ii++;
}

/**********************************************************/
/*                    Madgwick filter                     */
/**********************************************************/

inline void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}
  
  
  
 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones. 
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;   

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

