//xbee.ino

int led = 13;
char c;

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  Serial3.begin(9600);
}

void loop() {
  	// put your main code here, to run repeatedly:
  	digitalWrite(led, LOW);
  	if (Serial.available()){
  		c = (char)Serial.read();
  		Serial3.write(c);
  		digitalWrite(led, HIGH);
  	}
  	if (Serial3.available()){
  		c = (char)Serial3.read();
  		Serial.write(c);
  		digitalWrite(led, HIGH);
  	}
  	delay(10);
}