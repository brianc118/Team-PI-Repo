//HardwareSerialSend.ino

HardwareSerial uart = HardwareSerial();

int led = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  uart.begin(115200);
  Serial.begin(9600);
}

void loop() {
  uart.println("hi man");
  while(uart.available()){
  	uint8_t c = uart.read();
  	Serial.print(c);
  	Serial.print('\t');
  }
  Serial.println('e');
}
