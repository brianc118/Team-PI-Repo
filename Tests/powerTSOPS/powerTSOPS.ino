//powerTSOPS.ino

int led = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(33, INPUT);
  pinMode(led, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(32, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  	digitalWriteFast(25, HIGH);
  	digitalWriteFast(32, HIGH);
    digitalWrite(led, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(1000);               // wait for a second
    digitalWriteFast(25, LOW);
  	digitalWriteFast(32, LOW);
    digitalWrite(led, LOW);   // turn the LED off by making the voltage LOW
    delay(1000);               // wait for a second
}
