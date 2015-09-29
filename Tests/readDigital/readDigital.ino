//readDigital.ino

int led = 13;
#define INT1XM 5 // INT1XM tells us when accel data is ready
#define INT2XM 9 // INT2XM tells us when mag data is ready
#define DRDYG  8 // DRDYG  tells us when gyro data is ready

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
    if (digitalRead(DRDYG)){
    	Serial.println("readgyro");
    }
}
