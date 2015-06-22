uint8_t byteRead;

void setup() {                
  Serial.begin(9600);
  Serial1.begin(115200);
}

void loop() {
   /*  check if data has been sent from the computer: */
  if (Serial1.available()) {
    /* read the most recent byte */
    byteRead = Serial1.read();
    /*ECHO the value that was read, back to the serial port. */
    Serial.write(byteRead);
  }
}