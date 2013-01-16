#include <avr/sleep.h>
#include <avr/wdt.h>
#include <PinChangeInt.h>
#include <Wire.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usbdrv.h"
#include "oddebug.h"

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define BUTTON 14
#define LIS331HH_ADDRESS 25
#define MPL3115A2_ADDRESS 96

//#define BYPASS_TIMER_ISR 0

int RedLED = 5;
int GreenLED = 6;
int BlueLED = 9;
int Piezo = 10;
int i = 0;
int freq = 0;

volatile boolean f_wdt = 1;
volatile byte state = 0;
volatile byte alt_init = 0;
volatile byte acc_init = 0;
byte MPL3115A2_val = 0;
byte MPL3115A2_pval_msb = 0;
byte MPL3115A2_pval_csb = 0;
byte MPL3115A2_pval_lsb = 0;
byte MPL3115A2_bar_lsb = 0;
byte MPL3115A2_bar_msb = 0;
byte MPL3115A2_aval_msb = 0;
byte MPL3115A2_aval_csb = 0;
byte MPL3115A2_aval_lsb = 0;
uint16_t MPL3115A2_pval = 0;
uint16_t MPL3115A2_aval = 0;
uint16_t LIS331HH_X_val = 0;
byte LIS331HH_X_high = 0;
byte LIS331HH_X_low = 0;

void setup(){
  cbi(ADCSRA,ADEN);                    // switch Analog to Digital converter OFF
  cbi(PRR,PRTIM1);
  cbi(PRR,PRTIM0);
  //cbi(PRR,PRUSI);
  cbi(PRR,PRADC);
  pinMode(RedLED,INPUT);
  pinMode(GreenLED,INPUT);
  pinMode(BlueLED,INPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
  PCintPort::attachInterrupt(BUTTON, &state_change, FALLING);
  Wire.begin();
}

void loop(){
  switch (state) {
    case 0:
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
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);
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
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      f_wdt=0;
      for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5) { 
        // sets the value (range from 0 to 255):
        analogWrite(GreenLED, fadeValue);         
        // wait for 30 milliseconds to see the dimming effect    
        delay(5);                            
      } 
      // fade out from max to min in increments of 5 points:
      for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=5) { 
        // sets the value (range from 0 to 255):
        analogWrite(GreenLED, fadeValue);         
        // wait for 30 milliseconds to see the dimming effect    
        delay(5);                            
      }
    }
    system_sleep();
    break;
    case 3:
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      f_wdt=0;
      //tone(10, random(100, 15000), 200);
      tone(10, 4000);
      delay(200);
    }
    system_sleep();
    break;
    case 4:
    if (!alt_init){
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x26);
      Wire.write(0b00111010);
      Wire.endTransmission();
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x01);
      Wire.endTransmission(false);
      Wire.requestFrom(0x60, 3);    // request 3 bytes from slave device
      while(Wire.available()){    // slave may send less than requested 
        MPL3115A2_val = Wire.read();//dummy read
        MPL3115A2_pval_msb = Wire.read();  
        MPL3115A2_pval_csb = Wire.read();
        MPL3115A2_pval_lsb = Wire.read();
      }
      MPL3115A2_bar_msb = MPL3115A2_pval_msb;
      MPL3115A2_bar_msb = (MPL3115A2_bar_msb << 2);
      MPL3115A2_bar_msb = MPL3115A2_bar_msb | (MPL3115A2_pval_csb >> 6);
      MPL3115A2_bar_lsb = (MPL3115A2_pval_csb << 2);
      MPL3115A2_bar_lsb = MPL3115A2_pval_lsb | (MPL3115A2_pval_lsb >> 6);
      
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x14);
      Wire.write(MPL3115A2_bar_msb);
      Wire.endTransmission();
      
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x15);
      Wire.write(MPL3115A2_bar_lsb);
      Wire.endTransmission();
      
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x13);
      Wire.write(0x07);
      Wire.endTransmission();
   
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x26);
      Wire.write(0b10111001); //set active
      Wire.endTransmission();
      alt_init = 1;
//      UsbKeyboard.update();
//      UsbKeyboard.sendKeyStroke(KEY_A, MOD_SHIFT_LEFT);
//      UsbKeyboard.sendKeyStroke(KEY_L);
//      UsbKeyboard.sendKeyStroke(KEY_T);
//      UsbKeyboard.sendKeyStroke(KEY_S, MOD_SHIFT_LEFT);
//      UsbKeyboard.sendKeyStroke(KEY_T);
//      UsbKeyboard.sendKeyStroke(KEY_I);
//      UsbKeyboard.sendKeyStroke(KEY_C);
//      UsbKeyboard.sendKeyStroke(KEY_K);
//      
//      UsbKeyboard.sendKeyStroke(KEY_SPACE);
//  
//      UsbKeyboard.sendKeyStroke(KEY_B, MOD_SHIFT_LEFT);
//      UsbKeyboard.sendKeyStroke(KEY_Y);
//      
//      UsbKeyboard.sendKeyStroke(KEY_SPACE);
//      
//      UsbKeyboard.sendKeyStroke(KEY_P, MOD_SHIFT_LEFT);
//      UsbKeyboard.sendKeyStroke(KEY_A);
//      UsbKeyboard.sendKeyStroke(KEY_U);
//      UsbKeyboard.sendKeyStroke(KEY_L);
//      
//      UsbKeyboard.sendKeyStroke(KEY_SPACE);
//      
//      UsbKeyboard.sendKeyStroke(KEY_M, MOD_SHIFT_LEFT);
//      UsbKeyboard.sendKeyStroke(KEY_A);
//      UsbKeyboard.sendKeyStroke(KEY_R);
//      UsbKeyboard.sendKeyStroke(KEY_T);
//      UsbKeyboard.sendKeyStroke(KEY_I);
//      UsbKeyboard.sendKeyStroke(KEY_S);
    }
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      f_wdt=0;
//      UsbKeyboard.update();
      Wire.beginTransmission(MPL3115A2_ADDRESS);
      Wire.write(0x00);
      Wire.endTransmission(false);
      Wire.requestFrom(0x60, 1);    // request 1 byte from slave device
      while(Wire.available()){    // slave may send less than requested 
        MPL3115A2_val = Wire.read();  //dummy read
        MPL3115A2_val = Wire.read();    // receive a byte as character
      }
      if ((MPL3115A2_val & 0x08)>0){
        delay(250);
        digitalWrite(GreenLED,HIGH);
        delay(250);
        digitalWrite(GreenLED,LOW);
      }
      else {
        delay(250);
        digitalWrite(RedLED,HIGH);
        delay(250);
        digitalWrite(RedLED,LOW);
      }
    }
    system_sleep();
    break;
    case 5:
    if (!acc_init){
      digitalWrite(RedLED,LOW);
      digitalWrite(GreenLED,LOW);
      digitalWrite(BlueLED,LOW);
      pinMode(RedLED,INPUT);
      pinMode(GreenLED,INPUT);
      pinMode(BlueLED,INPUT);
      Wire.beginTransmission(LIS331HH_ADDRESS);
      Wire.write(0x20);
      Wire.write(0b10101001);
      Wire.endTransmission();
      acc_init = 1;
    }
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      f_wdt=0;
    Wire.beginTransmission(LIS331HH_ADDRESS);
    Wire.write(0xA8);
    Wire.endTransmission();
    Wire.requestFrom(LIS331HH_ADDRESS, 2);    // request 1 byte from slave device
    while(Wire.available()){    // slave may send less than requested 
      LIS331HH_X_low = Wire.read();
      LIS331HH_X_high = Wire.read();
    }
    LIS331HH_X_val = LIS331HH_X_high;
    LIS331HH_X_val = LIS331HH_X_val << 8;
    LIS331HH_X_val = LIS331HH_X_val | LIS331HH_X_low;
    if ((LIS331HH_X_val < 65336)&&(LIS331HH_X_val > 32767)) {
      pinMode(RedLED,OUTPUT); // set all ports into state before sleep
      digitalWrite(RedLED,HIGH);  // let led blink
      delay(10);
      digitalWrite(RedLED,LOW);
      pinMode(RedLED,INPUT);
      //analogWrite(RedLED,(65536-LIS331HH_X_val)/100);
      //digitalWrite(GreenLED,LOW);
      //digitalWrite(BlueLED,LOW);
      //pinMode(GreenLED,INPUT);
      //pinMode(BlueLED,INPUT);
    }
    if ((LIS331HH_X_val > 200)&&(LIS331HH_X_val < 32767)) {
      //digitalWrite(RedLED,LOW);
      //digitalWrite(GreenLED,LOW);
      //analogWrite(BlueLED,LIS331HH_X_val/100);
      pinMode(BlueLED,OUTPUT); // set all ports into state before sleep
      digitalWrite(BlueLED,HIGH);  // let led blink
      delay(10);
      digitalWrite(BlueLED,LOW);
      pinMode(BlueLED,INPUT);
      //analogWrite(RedLED,(65536-LIS331HH_X_val)/100);
      //digitalWrite(GreenLED,LOW);
      //digitalWrite(RedLED,LOW);
      //pinMode(GreenLED,INPUT);
      //pinMode(RedLED,INPUT);
    }
     if ((LIS331HH_X_val < 200)||(LIS331HH_X_val > 65336)) {
      //digitalWrite(BlueLED,LOW);
      //digitalWrite(RedLED,LOW);
      //pinMode(BlueLED,INPUT);
      //pinMode(RedLED,INPUT);
      pinMode(GreenLED,OUTPUT); // set all ports into state before sleep
      digitalWrite(GreenLED,HIGH);  // let led blink
      delay(10);
      digitalWrite(GreenLED,LOW);
      pinMode(GreenLED,INPUT);
      //analogWrite(RedLED,(65536-LIS331HH_X_val)/100);
    }
    }system_sleep();
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
    case 0:
      state = 1;
      setup_watchdog(7);
      break;
    case 1:
      state = 2;
      setup_watchdog(7);
      break;
    case 2:
      state = 3;
      setup_watchdog(8);
      break;
    case 3:
      state = 4;
      alt_init = 0;
      setup_watchdog(6);
      pinMode(RedLED,OUTPUT);
      pinMode(GreenLED,OUTPUT);
      pinMode(BlueLED,OUTPUT);
      break;
    case 4:
      state = 5;
      acc_init = 0;
      //wdt_disable();
      setup_watchdog(4);
      break;
    case 5:
      state = 0;
      wdt_disable();
      break;
  }
}
