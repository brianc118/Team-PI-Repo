#define RCM_SRS0_WAKEUP                     0x01
#define RCM_SRS0_LVD                        0x02
#define RCM_SRS0_LOC                        0x04
#define RCM_SRS0_LOL                        0x08
#define RCM_SRS0_WDOG                       0x20
#define RCM_SRS0_PIN                        0x40
#define RCM_SRS0_POR                        0x80

#define RCM_SRS1_LOCKUP                     0x02
#define RCM_SRS1_SW                         0x04
#define RCM_SRS1_MDM_AP                     0x08
#define RCM_SRS1_SACKERR                    0x20

volatile uint32_t i = 0;
IntervalTimer wdTimer;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  // You have about 6 secs to open the serial monitor before a watchdog reset
  //while(!Serial);
  Serial.begin(115200);
  Serial.println("just started");
  delay(100);
  printResetType();
  wdTimer.begin(KickDog, 4500000); // kick the dog every 500msec
}

void loop() {
  Serial.println(i++);
  delay(100);
}

void KickDog() {
  Serial.println("Kicking the dog!");
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

void printResetType() {
    if (RCM_SRS1 & RCM_SRS1_SACKERR)   Serial.println("[RCM_SRS1] - Stop Mode Acknowledge Error Reset");
    if (RCM_SRS1 & RCM_SRS1_MDM_AP)    Serial.println("[RCM_SRS1] - MDM-AP Reset");
    if (RCM_SRS1 & RCM_SRS1_SW)        Serial.println("[RCM_SRS1] - Software Reset");
    if (RCM_SRS1 & RCM_SRS1_LOCKUP)    Serial.println("[RCM_SRS1] - Core Lockup Event Reset");
    if (RCM_SRS0 & RCM_SRS0_POR)       Serial.println("[RCM_SRS0] - Power-on Reset");
    if (RCM_SRS0 & RCM_SRS0_PIN)       Serial.println("[RCM_SRS0] - External Pin Reset");
    if (RCM_SRS0 & RCM_SRS0_WDOG)      Serial.println("[RCM_SRS0] - Watchdog(COP) Reset");
    if (RCM_SRS0 & RCM_SRS0_LOC)       Serial.println("[RCM_SRS0] - Loss of External Clock Reset");
    if (RCM_SRS0 & RCM_SRS0_LOL)       Serial.println("[RCM_SRS0] - Loss of Lock in PLL Reset");
    if (RCM_SRS0 & RCM_SRS0_LVD)       Serial.println("[RCM_SRS0] - Low-voltage Detect Reset");
}

#ifdef __cplusplus
extern "C" {
#endif
  void startup_early_hook() {
    WDOG_TOVALL = (1000); // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
    WDOG_TOVALH = 0;
    WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN); // Enable WDG
    //WDOG_PRESC = 0; // prescaler 
  }
#ifdef __cplusplus
}
#endif