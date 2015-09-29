//teensylogic.ino

#define SOFTWARE_UPDATE_INTERVAL 100 // time between each update in us 18
#define ENCXA       3  //|
#define ENCXB       4  //| Use built in hardware quadrature interface
#define ENCYA       32 //|
#define ENCYB       25 //|
#define ENCZA       19
#define ENCZB       18
#define ENCEA       21
#define ENCEB       22

int A, B;
int main(void){
	Serial.begin(115200);
	pinMode(ENCZA, INPUT);
	pinMode(ENCZB, INPUT);
	while(true){
		A = digitalRead(ENCZA);
		B = digitalRead(ENCZB);
		Serial.print(A);
		Serial.print('\t');
		Serial.println(B);
		delay(10);
	}
}