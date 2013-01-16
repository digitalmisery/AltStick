#include <avr/sleep.h>
#include <avr/wdt.h>
#include <PinChangeInt.h>
#include <Wire.h>
#include "UsbKeyboard.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define BUTTON 14
#define LIS331HH_ADDRESS 25
#define MPL3115A2_ADDRESS 96

int RedLED = 5;
int GreenLED = 6;
int BlueLED = 9;
int Piezo = 10;
int i = 0;
int updateCount = 75;
volatile boolean f_wdt = 1;
volatile byte state = 0;
volatile byte alt_init = 0;
volatile byte acc_init = 0;
volatile byte USB_init = 0;
volatile byte USB_conn = 0;
volatile byte keyboardOutput = 0;
byte MPL3115A2_val = 0;
byte MPL3115A2_pval_msb = 0;
byte MPL3115A2_pval_csb = 0;
byte MPL3115A2_pval_lsb = 0;
byte MPL3115A2_bar_lsb = 0;
byte MPL3115A2_bar_msb = 0;
byte MPL3115A2_aval_msb = 0;
byte MPL3115A2_aval_csb = 0;
byte MPL3115A2_aval_lsb = 0;
uint32_t MPL3115A2_pval = 0;
uint16_t MPL3115A2_aval = 0;
float MPL3115A2_zeroed_aval = 0;
uint8_t MPL3115A2_afrac = 0;
uint16_t MPL3115A2_curr_aval = 0;
uint8_t MPL3115A2_curr_afrac = 0;
uint16_t MPL3115A2_max_aval = 0;
uint8_t MPL3115A2_max_afrac = 0;
int MPL3115A2_height_feet = 0;
uint16_t LIS331HH_X_val = 0;
byte LIS331HH_X_high = 0;
byte LIS331HH_X_low = 0;
boolean MPL3115A2_Ready = 0;
 
void setup(){
  cbi(ADCSRA,ADEN);                    // switch Analog to Digital converter OFF
  cbi(PRR,PRTIM1);
  cbi(PRR,PRTIM0);
  //cbi(PRR,PRUSI);
  cbi(PRR,PRADC);
  
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);
  pinMode(7, INPUT);
  pinMode(RedLED,INPUT);
  pinMode(GreenLED,INPUT);
  pinMode(BlueLED,INPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
  PCintPort::attachInterrupt(BUTTON, &state_change, FALLING);
  Wire.begin();
}

void loop(){
  
  if ((USB_conn == 0) && (state == 7))
    state = 0;
  if (USB_conn == 1)
    state = 7;
  switch (state) {
    case 0:
    wdt_disable();
    digitalWrite(RedLED,LOW);
    digitalWrite(GreenLED,LOW);
    digitalWrite(BlueLED,LOW);
    pinMode(RedLED,INPUT);
    pinMode(GreenLED,INPUT);
    pinMode(BlueLED,INPUT);
    Wire.beginTransmission(LIS331HH_ADDRESS);
    Wire.write(0x20);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(MPL3115A2_ADDRESS);
    Wire.write(0x26);
    Wire.write(0xB8);
    Wire.endTransmission();
    digitalWrite(2, HIGH);
    EIMSK |= _BV(INT0);  
    sei();
    system_sleep();
    break;
    case 1:
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      f_wdt=0;
      switch (i) {
        case 0:
          i++;
          pinMode(RedLED,OUTPUT); // set all ports into state before sleep
          digitalWrite(RedLED,HIGH);  // let led blink
          delay(10);
          digitalWrite(RedLED,LOW);
          pinMode(RedLED,INPUT); // set all used port to intput to save power
          system_sleep();
          break;
        case 1:
          i++;
          //i=0;
          pinMode(GreenLED,OUTPUT); // set all ports into state before sleep
          digitalWrite(GreenLED,HIGH);  // let led blink
          delay(10);
          digitalWrite(GreenLED,LOW);
          pinMode(GreenLED,INPUT); // set all used port to intput to save power
          system_sleep();
          break;
        case 2:
          i=0;
          pinMode(BlueLED,OUTPUT); // set all ports into state before sleep
          digitalWrite(BlueLED,HIGH);  // let led blink
          delay(10);
          digitalWrite(BlueLED,LOW);
          pinMode(BlueLED,INPUT); // set all used port to intput to save power
          system_sleep();
          break;
      }
    }
    break;
    case 2:
//      for (int i=0; i<2; i++) {
      pinMode(RedLED,OUTPUT); // set all ports into state before sleep
      digitalWrite(RedLED,HIGH);  // let led blink
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x26);
      Wire.write(0b10111000); //set alt
      Wire.endTransmission();
      
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x13);
      Wire.write(0x07);
      Wire.endTransmission();
   
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x26);
      Wire.write(0b10111001); //set active
      Wire.endTransmission();
      while (!MPL3115A2_Ready){
        Wire.beginTransmission(MPL3115A2_ADDRESS);
        Wire.write(0x00);
        Wire.endTransmission(false);
        Wire.requestFrom(MPL3115A2_ADDRESS, 1);
        MPL3115A2_Ready = Wire.read() & 0x08;
      }
      
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x01);
      Wire.endTransmission(false);
      Wire.requestFrom(MPL3115A2_ADDRESS, 3);    // request 3 bytes from slave device
      while(Wire.available()){    // slave may send less than requested 
        MPL3115A2_aval_msb = Wire.read();  
        MPL3115A2_aval_csb = Wire.read();
        MPL3115A2_aval_lsb = Wire.read();
      }
      MPL3115A2_aval = MPL3115A2_aval_msb;
      MPL3115A2_aval = MPL3115A2_aval << 8;
      MPL3115A2_aval = MPL3115A2_aval | MPL3115A2_aval_csb;
      MPL3115A2_afrac = MPL3115A2_aval_lsb >> 4;
      MPL3115A2_zeroed_aval = MPL3115A2_aval + MPL3115A2_afrac*0.0625;
      digitalWrite(RedLED,LOW);
      pinMode(RedLED,INPUT); // set all used port to intput to save power 
//      }
      system_sleep();
    break;
    case 3:
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      f_wdt=0;
      pinMode(GreenLED,OUTPUT); // set all ports into state before sleep
      digitalWrite(GreenLED,HIGH);  // let led blink
      while (!MPL3115A2_Ready){
        Wire.beginTransmission(MPL3115A2_ADDRESS);
        Wire.write(0x00);
        Wire.endTransmission(false);
        Wire.requestFrom(MPL3115A2_ADDRESS, 1);
        MPL3115A2_Ready = Wire.read() & 0x08;
      }
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x01);
      Wire.endTransmission(false);
      Wire.requestFrom(MPL3115A2_ADDRESS, 3);    // request 3 bytes from slave device
      while(Wire.available()){    // slave may send less than requested 
         //MPL3115A2_val = Wire.read();//dummy read
        MPL3115A2_aval_msb = Wire.read();  
        MPL3115A2_aval_csb = Wire.read();
        MPL3115A2_aval_lsb = Wire.read();
      }
      MPL3115A2_curr_aval = MPL3115A2_aval_msb;
      MPL3115A2_curr_aval = MPL3115A2_curr_aval << 8;
      MPL3115A2_curr_aval = MPL3115A2_curr_aval | MPL3115A2_aval_csb;
      MPL3115A2_curr_afrac = MPL3115A2_aval_lsb >> 4;
      if (MPL3115A2_curr_aval+MPL3115A2_curr_afrac*0.0625 > MPL3115A2_max_aval+MPL3115A2_max_afrac*0.0625){
        MPL3115A2_max_aval = MPL3115A2_curr_aval;
        MPL3115A2_max_afrac = MPL3115A2_curr_afrac;
      }
//      delay(10);
      digitalWrite(GreenLED,LOW);
      pinMode(GreenLED,INPUT); // set all used port to intput to save power 
      system_sleep();
    }
    break;
    case 4:
    if ((((MPL3115A2_max_aval+MPL3115A2_max_afrac*0.0625)-MPL3115A2_zeroed_aval)*3.281) >=0)
        MPL3115A2_height_feet = (int)floor(((MPL3115A2_max_aval+MPL3115A2_max_afrac*0.0625)-MPL3115A2_zeroed_aval)*3.281);
    else MPL3115A2_height_feet = 0;
    system_sleep();
    break;
    case 5:
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      f_wdt=0;
      pinMode(BlueLED,OUTPUT);
      for (int i=0; i < MPL3115A2_height_feet; i++) {
        digitalWrite(BlueLED,HIGH);  // let led blink
        delay(20);
        digitalWrite(BlueLED,LOW);
        delay(250);
      }
      pinMode(BlueLED,INPUT); // set all used port to intput to save power
      system_sleep();     
    }
    break;
    case 7:
    if (!USB_init){
      EIMSK &= (~_BV(INT0));
      digitalWrite(2, LOW);
      UsbKeyboard.initializeKeyboard();
      delay(10);
      UsbKeyboard.reconnectKeyboard();
      USB_init = 1;
      setup_watchdog(0);        
    }
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      f_wdt=0; 
      UsbKeyboard.update();
      updateCount--;
    }
    if (updateCount == 0){
      updateCount = 50;
      pinMode(GreenLED,OUTPUT); // set all ports into state before sleep
      digitalWrite(GreenLED,HIGH);  // let led blink
      delay(10);
      digitalWrite(GreenLED,LOW);
      pinMode(GreenLED,INPUT);    
      system_sleep();
   }
    if (keyboardOutput == 1){  
      UsbKeyboard.typeCharacters("Height:");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
//      UsbKeyboard.typeCharacters("M ");
//      UsbKeyboard.typeNumber((MPL3115A2_max_aval - MPL3115A2_aval));
//      UsbKeyboard.sendKeyStroke(KEY_SPACE);
//      UsbKeyboard.typeNumber((MPL3115A2_max_afrac - MPL3115A2_afrac));
//      UsbKeyboard.sendKeyStroke(KEY_SPACE);
//      if ((((MPL3115A2_max_aval+MPL3115A2_max_afrac*0.0625)-MPL3115A2_zeroed_aval)*3.281) >=0)
//        UsbKeyboard.typeNumber((int)floor((((MPL3115A2_max_aval+MPL3115A2_max_afrac*0.0625)-MPL3115A2_zeroed_aval)*3.281)));
//        else UsbKeyboard.typeNumber(0);
      UsbKeyboard.typeNumber(MPL3115A2_height_feet);
      UsbKeyboard.typeCharacters("F");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeCharacters("Zeroed: ");
//      UsbKeyboard.typeNumber((int)MPL3115A2_zeroed_aval);
      UsbKeyboard.typeFloat(MPL3115A2_zeroed_aval*3.281);
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeCharacters("Max: ");
//      UsbKeyboard.typeNumber((int)(MPL3115A2_max_aval+MPL3115A2_max_afrac*0.0625));
      UsbKeyboard.typeFloat((MPL3115A2_max_aval+MPL3115A2_max_afrac*0.0625)*3.281);
      UsbKeyboard.sendKeyStroke(KEY_ENTER);
      keyboardOutput = 0;
    }
    break;    
  }
}

// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  if (digitalRead(2) == LOW) {
      USB_conn = 1;
      if (USB_init) {}
      else USB_init = 0;
  }else {
    USB_conn = 0;
    USB_init = 0;
  }
  //sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}
  
// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}

void state_change () {
  switch (state) {
    case 0:  //standby
      state = 1;
      setup_watchdog(7);
      break;
    case 1:  //LEDs flash
      state = 2;
      wdt_disable();
      break;
    case 2: //zero height
      state = 3;
      setup_watchdog(5);
      break;
    case 3: //acquire height
      state = 4;
      wdt_disable();
      break;
    case 4: //stop aquiring height
      state = 5;
      f_wdt=1;
      setup_watchdog(9);
      break;
    case 5: //flash out height
      state = 0;
      wdt_disable();
      break;
    case 7:
      keyboardOutput = 1;
  }
}
