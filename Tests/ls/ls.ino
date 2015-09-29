//ls.ino

int led = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(A12, INPUT);
  digitalWrite(26, HIGH);
}


void loop() {
	Serial.println(analogRead(A12));
	delay(500);
}
