//cosBenchmark.ino
float result;
unsigned long start, end;
unsigned long overhead;
int i = 0;
void setup() {
}

void loop() {
	start = micros();
	for (i = 0; i < 100000; i++){
	}
	for (i = 0; i < 100000; i++){
	}
	for (i = 0; i < 100000; i++){
	}
	for (i = 0; i < 100000; i++){
	}
	for (i = 0; i < 100000; i++){
	}
	for (i = 0; i < 100000; i++){
	}
	for (i = 0; i < 100000; i++){
	}
	for (i = 0; i < 100000; i++){
	}
	end = micros();
	overhead = end - start;

	start = micros();

	for (i = 0; i < 100000; i++){
		result = cos(0.81887902);
	}
	for (i = 0; i < 100000; i++){
		result = cos(0.71887902);
	}
	for (i = 0; i < 100000; i++){
		result = cos(0.61887902);
	}
	for (i = 0; i < 100000; i++){
		result = cos(0.51887902);
	}
	for (i = 0; i < 100000; i++){
		result = cos(0.41887902);
	}
	for (i = 0; i < 100000; i++){
		result = cos(0.31887902);
	}
	for (i = 0; i < 100000; i++){
		result = cos(0.21887902);
	}
	for (i = 0; i < 100000; i++){
		result = cos(0.11887902);
	}
	end = micros();

	Serial.println("complete");
	Serial.println((end - start), 6);
}
