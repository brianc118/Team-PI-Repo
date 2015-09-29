/*
  leo_usb2serial
  Allows to use an Arduino Leonardo as an usb to serial converter.
 */
static long baud = 57600;
static long newBaud = baud;

// this pin will output the DTR signal (as set by the pc)
#define DTR_PIN 13

#define LINESTATE_DTR  1

void lineCodingEvent(long baud, byte databits, byte parity, byte charFormat)
{
  newBaud = baud;
}

void lineStateEvent(unsigned char linestate)
{
  if(linestate & LINESTATE_DTR)
    digitalWrite(DTR_PIN, HIGH);
  else
    digitalWrite(DTR_PIN, LOW);
}

void setup() {
  pinMode(DTR_PIN, OUTPUT);
  digitalWrite(DTR_PIN, LOW);
  Serial.begin(baud);
  Serial3.begin(baud);
}

void loop() {
	digitalWriteFast(13, LOW);
  // Set the new baud rate
  if(newBaud != baud) {
    baud = newBaud;
    Serial3.end();
    Serial3.begin(baud);
  }

  // copy from virtual serial line to uart and vice versa
  if (Serial.available()) {
    char c = (char)Serial.read();
    Serial3.write(c);
    digitalWriteFast(13, HIGH);
  }
  if (Serial3.available()) {
    char c = (char)Serial3.read();
    Serial.write(c);
  }
}