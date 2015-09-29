//serial.ino

int led = 13;

void setup() {
 	Serial.begin(57600);
 	Serial1.begin(57600);
}

void loop() {
	while (Serial1.available()){
		Serial.write(Serial1.read());
	}
}
