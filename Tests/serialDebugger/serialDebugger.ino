//serialDebugger.ino

int led = 13;

void setup() {
	pinMode(13, OUTPUT);
	Serial.begin(9600);
	Serial1.begin(9600);
}

void loop() {
	while (Serial1.available()){
		Serial.print((char)Serial1.read());
		digitalWrite(13, HIGH);
	}
	digitalWrite(13, LOW);
}
