// arrayShift.ino

// primative array shifting function. Will port to macro in future for optimal speed

// function to shift array. + is moving elements down
static inline void arrayShift(int *a, int lower, int upper, bool up, int len) {
	int i;
	if (up){
		if (lower == 0){
			for (i = lower; i < upper; i++) {
				*(a + i) = *(a + i + 1);
			}
		}
		else{
			for (i = lower - 1; i < upper; i++) {
				*(a + i) = *(a + i + 1);
			}
		}
	}	
	else{
		if (upper == len - 1){
			for (i = upper - 1; i >= lower; i--){
				*(a + i + 1) = *(a + i);
			}
		}
		else{
			for (i = upper; i >= lower; i--){
				*(a + i + 1) = *(a + i);
			}
		}
	}	
}

// macro functions.
#define ARRAYSHIFTUP(a, lower, upper){            \
    if (lower == 0){                              \
        for (int q = lower; q < upper; q++){      \
            *(a + q) = *(a + q + 1); }            \
    } else{                                       \
        for (int q = lower - 1; q < upper; q++){  \
            *(a + q) = *(a + q + 1); }}}          \

#define ARRAYSHIFTDOWN(a, lower, upper){          \
    if (upper == (sizeof(a)/sizeof(a[0])) - 1){   \
        for (int q = upper - 1; q >= lower; q--){ \
            *(a + q + 1) = *(a + q); }            \
    } else{                                       \
        for (int q = upper; q >= lower; q--){     \
            *(a + q + 1) = *(a + q); }}}          \

#define TSOP_COUNT 24

	

int array[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

uint8_t filteredData[TSOP_COUNT] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 10, 11, 12, 13, 14, 15, 16, 17, 23, 22, 21, 20, 19, 18};
uint8_t filteredDataSorted[TSOP_COUNT] = {0};
uint8_t indexes[TSOP_COUNT] = {0};

#define PRINTARRAY(a){                                \
	Serial.print('{');                                \
	for (int i = 0; i < sizeof(a)/sizeof(a[0]); i++){ \
		Serial.print(a[i]);                           \
		Serial.print('\t');	}                         \
	Serial.println('}'); }                            \

void setup() {
	Serial.begin(9600);
	
}

void loop() {
	while (Serial.available() <= 0){
		delay(500);
	}
	while (Serial.available() > 0){
		Serial.read();
	}	//arrayShift(array, 2, 6, true, 10);
	//arrayShift(array, 0, 9, true, 10);
	ARRAYSHIFTDOWN(array, 0, 9);
	PRINTARRAY(array);


	delay(100);
	Serial.println("Beginning TSOP FILTER SIMULATION");
	Serial.println();

	for (int i = 0; i < TSOP_COUNT; i++){
		for (int j = 0; j < TSOP_COUNT; j++){
			if (filteredData[i] > filteredDataSorted[j]){
				// we've found our place!
				// shift elements from index j down
				if (j <= i){
					ARRAYSHIFTDOWN(filteredDataSorted, j, i);
					ARRAYSHIFTDOWN(indexes, j, i);
				}
				
				filteredDataSorted[j] = filteredData[i];
				indexes[j] = i;
				break;
			}
		}
	}
	PRINTARRAY(filteredData);
	PRINTARRAY(filteredDataSorted);
	PRINTARRAY(indexes);
	memset(filteredDataSorted, 0, sizeof(filteredDataSorted));
	memset(indexes, 0, sizeof(indexes));
	Serial.println(1 ^ 1);
	Serial.println(0 ^ 1);
}
