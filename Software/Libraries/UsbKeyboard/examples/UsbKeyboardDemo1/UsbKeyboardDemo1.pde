#include "UsbKeyboard.h"

#define BUTTON_PIN 14
#define RED_LED 5
#define GREEN_LED 6
#define BLUE_LED 9

long previousMillis = 0;        // will store last time LED was updated
long interval = 1000;           // interval at which to blink (milliseconds)

// If the timer isr is corrected
// to not take so long change this to 0.
#define BYPASS_TIMER_ISR 0

void setup() {
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);
  pinMode(RED_LED,OUTPUT);
  pinMode(GREEN_LED,OUTPUT);
  pinMode(BLUE_LED,OUTPUT);
  
#if BYPASS_TIMER_ISR
  // disable timer 0 overflow interrupt (used for millis)
  TIMSK0&=!(1<<TOIE0); // ++
#endif
}

#if BYPASS_TIMER_ISR
void delayMs(unsigned int ms) {
   /*
  */ 
  for (int i = 0; i < ms; i++) {
    delayMicroseconds(1000);
  }
}
#endif

void loop() {
  
  UsbKeyboard.update();
  
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;   
    digitalWrite(BLUE_LED, HIGH);
    delay(10);
    digitalWrite(BLUE_LED, LOW);
  }

  if (digitalRead(BUTTON_PIN) == 0) {
    digitalWrite(GREEN_LED, HIGH);
    
    //UsbKeyboard.sendKeyStroke(KEY_B, MOD_GUI_LEFT);
    
    UsbKeyboard.sendKeyStroke(KEY_A, MOD_SHIFT_LEFT);
    UsbKeyboard.sendKeyStroke(KEY_L);
    UsbKeyboard.sendKeyStroke(KEY_T);
    UsbKeyboard.sendKeyStroke(KEY_S, MOD_SHIFT_LEFT);
    UsbKeyboard.sendKeyStroke(KEY_T);
    UsbKeyboard.sendKeyStroke(KEY_I);
    UsbKeyboard.sendKeyStroke(KEY_C);
    UsbKeyboard.sendKeyStroke(KEY_K);
    
    UsbKeyboard.sendKeyStroke(KEY_SPACE);

    UsbKeyboard.sendKeyStroke(KEY_B, MOD_SHIFT_LEFT);
    UsbKeyboard.sendKeyStroke(KEY_Y);
    
    UsbKeyboard.sendKeyStroke(KEY_SPACE);
    
    UsbKeyboard.sendKeyStroke(KEY_P, MOD_SHIFT_LEFT);
    UsbKeyboard.sendKeyStroke(KEY_A);
    UsbKeyboard.sendKeyStroke(KEY_U);
    UsbKeyboard.sendKeyStroke(KEY_L);
    
    UsbKeyboard.sendKeyStroke(KEY_SPACE);
    
    UsbKeyboard.sendKeyStroke(KEY_M, MOD_SHIFT_LEFT);
    UsbKeyboard.sendKeyStroke(KEY_A);
    UsbKeyboard.sendKeyStroke(KEY_R);
    UsbKeyboard.sendKeyStroke(KEY_T);
    UsbKeyboard.sendKeyStroke(KEY_I);
    UsbKeyboard.sendKeyStroke(KEY_S);
    
    //UsbKeyboard.sendKeyStroke(KEY_B, MOD_GUI_LEFT);

    UsbKeyboard.sendKeyStroke(KEY_ENTER);
    
    digitalWrite(GREEN_LED, LOW);
#if BYPASS_TIMER_ISR  // check if timer isr fixed.
    delayMs(20);
#else
    delay(20);
#endif
   }
}