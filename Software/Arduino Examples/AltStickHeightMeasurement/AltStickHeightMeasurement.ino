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

#define REDLED 5
#define GREENLED 6
#define BLUELED 9
#define PIEZO 10
#define USB_D_PLUS 2
#define USB_D_MINUS 7
#define ALT_INT1 0
#define ALT_INT2 1
#define ACC_INT1 16
#define ACC_INT2 17
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
byte MPL3115A2_adelta_msb = 0;
byte MPL3115A2_adelta_csb = 0;
byte MPL3115A2_adelta_lsb = 0;
byte MPL3115A2_aval_msb = 0;
byte MPL3115A2_aval_csb = 0;
byte MPL3115A2_aval_lsb = 0;
byte MPL3115A2_amax_msb = 0;
byte MPL3115A2_amax_csb = 0;
byte MPL3115A2_amax_lsb = 0;
uint16_t MPL3115A2_aval = 0;
float MPL3115A2_aavg = 0;
float MPL3115A2_zeroed_aval = 0;
uint8_t MPL3115A2_afrac = 0;
uint16_t MPL3115A2_amax = 0;
uint8_t MPL3115A2_amaxfrac = 0;
unsigned int MPL3115A2_height_feet = 0;
uint16_t LIS331HH_X_val = 0;
byte LIS331HH_X_high = 0;
byte LIS331HH_X_low = 0;
boolean MPL3115A2_Ready = 0;
boolean launchDetected = false;
 
void setup(){
  cbi(ADCSRA,ADEN);                    // switch Analog to Digital converter OFF
  cbi(PRR,PRTIM1);
  cbi(PRR,PRTIM0);
  //cbi(PRR,PRUSI);
  cbi(PRR,PRADC);
  
  pinMode(USB_D_PLUS, INPUT); 
  digitalWrite(USB_D_PLUS, HIGH); // Turn on pull-up
  pinMode(USB_D_MINUS, INPUT); 
  pinMode(REDLED,INPUT);
  pinMode(GREENLED,INPUT);
  pinMode(BLUELED,INPUT);
  pinMode(BUTTON, INPUT);
  pinMode(ALT_INT1, INPUT);
  digitalWrite(ALT_INT1, HIGH);
  pinMode(ALT_INT2, INPUT);
  digitalWrite(ALT_INT2, HIGH);
  pinMode(ACC_INT1, INPUT);
  pinMode(ACC_INT2, INPUT);
  digitalWrite(BUTTON, HIGH);
  PCintPort::attachInterrupt(BUTTON, &state_change, FALLING);
  PCintPort::attachInterrupt(ACC_INT1, &high_acc, RISING);
  PCintPort::attachInterrupt(ACC_INT2, &low_acc, RISING);
  Wire.begin();
  Initialize_MPL3115A2();
  Initialize_LIS331HH();
}

void loop(){
  if ((USB_conn == 0) && (state == 7))
    state = 0;
  if (USB_conn == 1)
    state = 7;
  switch (state) {
    case 0: // Reset Low-Power State
      wdt_disable();
      Enable_LEDs(0);
      Sleep_LIS331HH();
      Sleep_MPL3115A2();
      digitalWrite(USB_D_PLUS, HIGH);
      EIMSK |= _BV(INT0);  
      sei(); // Enable interrups
      system_sleep(); // Go to sleep
      break;
    case 1: //Average 16 samples for zeroed value
      delay(1000);     
      Red_LED_On(1);
      Average_Alt_Data_MPL3115A2(16);
      MPL3115A2_zeroed_aval = MPL3115A2_aavg;
      Clear_Max_Alt_MPL3115A2();
      Red_LED_On(0);
      Activate_Lauch_Detection_LIS331HH();
      system_sleep();
      break;
    case 2: //Take Altitude Measurements with WDT
      if (launchDetected){
        Deactivate_Lauch_Detection_LIS331HH();
        launchDetected = false;
      }
      if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
        f_wdt=0;
        Activate_ZeroG_Detection_LIS331HH();
        Green_LED_On(1);
        Measure_Alt_MPL3115A2();
        Green_LED_On(0);
        system_sleep();
      }
      break;
    case 3: //Compute Max Height in feet
    //delay(1000);
    Sleep_LIS331HH();
    Sleep_MPL3115A2();
    //tone(10, 4000);
    Red_LED_On(1);
    Read_Max_Alt_MPL3115A2();
    if (MPL3115A2_amax-1 > MPL3115A2_zeroed_aval)
      MPL3115A2_height_feet = (int)floor((((MPL3115A2_amax-1)+MPL3115A2_amaxfrac*0.0625)-MPL3115A2_zeroed_aval)*3.281);
    else MPL3115A2_height_feet = 0;
    Red_LED_On(0);
    //delay(250);
    //noTone(10);
    system_sleep();
    break;
    case 4:
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      //f_wdt=0;
      if (MPL3115A2_height_feet/100 > 0) {
        pinMode(REDLED, OUTPUT);
        for (int i=0; i < MPL3115A2_height_feet/100; i++) {
          digitalWrite(REDLED,HIGH);  // let led blink
          delay(100);
          digitalWrite(REDLED,LOW);
          delay(250);
        }
        Red_LED_On(0);
        delay(500);
        pinMode(GREENLED, OUTPUT);
        for (int i=0; i < (MPL3115A2_height_feet/10)%10; i++) {
          digitalWrite(GREENLED,HIGH);  // let led blink
          delay(100);
          digitalWrite(GREENLED,LOW);
          delay(250);
        }
        Green_LED_On(0);
        delay(500);
        pinMode(BLUELED, OUTPUT);
        for (int i=0; i < (MPL3115A2_height_feet%100)%10; i++) {
          digitalWrite(BLUELED,HIGH);  // let led blink
          delay(100);
          digitalWrite(BLUELED,LOW);
          delay(250);
        }
        Blue_LED_On(0);
      }
      else
      if (MPL3115A2_height_feet/10 > 0) {
        pinMode(GREENLED, OUTPUT);
        for (int i=0; i < (MPL3115A2_height_feet/10); i++) {
          digitalWrite(GREENLED,HIGH);  // let led blink
          delay(100);
          digitalWrite(GREENLED,LOW);
          delay(250);
        }
        Green_LED_On(0);
        delay(500);
        pinMode(BLUELED, OUTPUT);
        for (int i=0; i < (MPL3115A2_height_feet)%10; i++) {
          digitalWrite(BLUELED,HIGH);  // let led blink
          delay(100);
          digitalWrite(BLUELED,LOW);
          delay(250);
        }
        Blue_LED_On(0);
      }
      else
      if (MPL3115A2_height_feet > 0) {
        pinMode(BLUELED,OUTPUT);
        for (int i=0; i < MPL3115A2_height_feet; i++) {
          digitalWrite(BLUELED,HIGH);  // let led blink
          delay(100);
          digitalWrite(BLUELED,LOW);
          delay(250);
        }
        Blue_LED_On(0);
      }      
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
      Green_LED_On(1);
      delay(10);
      Green_LED_On(0); 
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
      //UsbKeyboard.typeFloat((MPL3115A2_aavg)*3.281);
      UsbKeyboard.typeFloat((MPL3115A2_amax-1+MPL3115A2_amaxfrac*0.0625)*3.281);
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
      wdt_disable();
      break;
    case 1: //zero height
      state = 2;
      setup_watchdog(3);
      break;
    case 2: //acquire height
      state = 3;
      wdt_disable();
      break;
    case 3: //stop aquiring height
      state = 4;
      f_wdt=1;
      wdt_disable();
      //setup_watchdog(9);
      break;
    case 4: //flash out height
      //state = 0;
      //wdt_disable();
      break;
    case 7:
      keyboardOutput = 1;
  }
}

void high_acc () {
  switch (state) {
    case 0:  //standby
      break;
    case 1: //zero height
      state = 2;
      setup_watchdog(3);
      launchDetected = true;
      break;
    case 2: //acquire height
      break;
    case 3: //stop aquiring height
      break;
    case 4: //flash out height
      break;
    case 7:
      break;
  }
}

void low_acc () {
  switch (state) {
    case 0:  //standby
      break;
    case 1: //zero height
      break;
    case 2: //acquire height
      state = 3;
      wdt_disable();
      break;
    case 3: //stop aquiring height
      break;
    case 4: //flash out height
      break;
    case 7:
      break;
  }
}

void Enable_LEDs (boolean enable) {
  if (enable) {
    pinMode(REDLED,OUTPUT);
    pinMode(GREENLED,OUTPUT);
    pinMode(BLUELED,OUTPUT);
    digitalWrite(REDLED,LOW);
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,LOW);
  }
  else {
    digitalWrite(REDLED,LOW);
    digitalWrite(GREENLED,LOW);
    digitalWrite(BLUELED,LOW);
    pinMode(REDLED,INPUT);
    pinMode(GREENLED,INPUT);
    pinMode(BLUELED,INPUT);
  }
}

void Red_LED_On (boolean on) {
  if (on) {
    pinMode(REDLED,OUTPUT);
    digitalWrite(REDLED,HIGH);
  }
  else {
    digitalWrite(REDLED,LOW);
    pinMode(REDLED,INPUT);
  }
}

void Green_LED_On (boolean on) {
  if (on) {
    pinMode(GREENLED,OUTPUT);
    digitalWrite(GREENLED,HIGH);
  }
  else {
    digitalWrite(GREENLED,LOW);
    pinMode(GREENLED,INPUT);
  }
}

void Blue_LED_On (boolean on) {
  if (on) {
    pinMode(BLUELED,OUTPUT);
    digitalWrite(BLUELED,HIGH);
  }
  else {
    digitalWrite(BLUELED,LOW);
    pinMode(BLUELED,INPUT);
  }
}

void Initialize_MPL3115A2() {
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x26); //CTRL_REG1
  Wire.write(0b10100000); //Alt, 16x OS, Standby
  Wire.endTransmission();
  
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x13); //PT_DATA_CFG
  Wire.write(0x06); //Enable data ready flag for pressure
  Wire.endTransmission();
  
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x28); //CTRL_REG3 - Interrupts control
  Wire.write(0x11); //Active low and open drain
  Wire.endTransmission();
  
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x29); //CTRL_REG4 - Interrupts enabled
  Wire.write(0x80); //Data Ready and Pressure Change interrupts
  Wire.endTransmission();
  /*
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x2A); //CTRL_REG5 - Interrupt config
  Wire.write(0x80); //INT1 = Data Ready, INT2 = Pressure Change
  Wire.endTransmission();
  */
  Clear_Max_Alt_MPL3115A2();
}

void Sleep_MPL3115A2() {
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x26); //CTRL_REG1
  Wire.write(0xA0); //STANDBY 
  Wire.endTransmission();
}

void Measure_Alt_MPL3115A2 () {
  //Wire.beginTransmission(MPL3115A2_ADDRESS);
  //Wire.write(0x26); //CTRL_REG1
  //Wire.endTransmission(false);
  //Wire.requestFrom(MPL3115A2_ADDRESS, 1);
  //Wire.read();
  //Wire.endTransmission();
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x26); //CTRL_REG1
  Wire.write(0b10100010); //Alt, 32x OS, One-Shot, Standby
  Wire.endTransmission();
}

boolean Data_Ready_MPL3115A2 () { 
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x00); //Status register
  Wire.endTransmission(false);
  Wire.requestFrom(MPL3115A2_ADDRESS, 1);
  MPL3115A2_Ready = Wire.read() & 0x04; //New pressure data ready
  return MPL3115A2_Ready;
}

void Read_Alt_Data_MPL3115A2() {
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x01); //Pressure/Alt Data Out MSB - auto increments
  Wire.endTransmission(false); //Keep bus active
  Wire.requestFrom(MPL3115A2_ADDRESS, 3); // request 3 bytes
  while(Wire.available()){  // slave may send less than requested
    MPL3115A2_aval_msb = Wire.read();  
    MPL3115A2_aval_csb = Wire.read();
    MPL3115A2_aval_lsb = Wire.read();
  }
  MPL3115A2_aval = MPL3115A2_aval_msb;
  MPL3115A2_aval = MPL3115A2_aval << 8;
  MPL3115A2_aval = MPL3115A2_aval | MPL3115A2_aval_csb;
  MPL3115A2_afrac = MPL3115A2_aval_lsb >> 4;
}

void Average_Alt_Data_MPL3115A2 (int samples) {
  for (int i=0; i<samples; i++){
    Measure_Alt_MPL3115A2();
    while (!Data_Ready_MPL3115A2()){}
    //while(digitalRead(ALT_INT1) == HIGH){} //Wait until INT1 goes low
    Read_Alt_Data_MPL3115A2();
    MPL3115A2_aavg = MPL3115A2_aavg + MPL3115A2_aval+MPL3115A2_afrac*0.0625;
  }
  MPL3115A2_aavg = MPL3115A2_aavg/samples;
}

void Clear_Max_Alt_MPL3115A2 () {
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x21); //P_MAX_MSB - auto increments
  Wire.write(0x00); //Zero out value in MSB
  Wire.write(0x00); //Zero out value in CSB
  Wire.write(0x00); //Zero out value in LSB
  Wire.endTransmission();
}

void Read_Max_Alt_MPL3115A2 () {
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x21); //P_MAX_MSB - auto increments
  Wire.endTransmission(false); //Keep bus active
  Wire.requestFrom(MPL3115A2_ADDRESS, 3); // request 3 bytes
  while(Wire.available()){  // slave may send less than requested
    MPL3115A2_amax_msb = Wire.read();  
    MPL3115A2_amax_csb = Wire.read();
    MPL3115A2_amax_lsb = Wire.read();
  }
  MPL3115A2_amax = MPL3115A2_amax_msb;
  MPL3115A2_amax = MPL3115A2_amax << 8;
  MPL3115A2_amax = MPL3115A2_amax | MPL3115A2_amax_csb;
  MPL3115A2_amaxfrac = MPL3115A2_amax_lsb >> 4;
}

void Initialize_LIS331HH() {
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x20); //CTRL_REG1
  Wire.write(0b11000111); //Low Power 10Hz, axes enabled
  Wire.endTransmission();
  
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x23); //CTRL_REG4
  //Wire.write(0b00000000); // +/- 6g
  Wire.write(0b00110000); // +/- 24g
  //Wire.write(0b00010000); // +/- 12g
  Wire.endTransmission();
  
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x30); //INT1_CFG
  Wire.write(0b00000000); //Disabled
  Wire.endTransmission();
  
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x34); //INT2_CFG
  Wire.write(0b00000000); //Disabled
  Wire.endTransmission();
}

void Sleep_LIS331HH() {
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x20); //CTRL_REG1
  Wire.write(0x00); //Power down, axes disabled
  Wire.endTransmission();
}

// INT1 will signal launch when acceleration exceeds threshold
void Activate_Lauch_Detection_LIS331HH() {
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x20); //CTRL_REG1
  Wire.write(0b00100111); //On, 50Hz, axes enabled
  Wire.endTransmission();
  
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x30); //INT1_CFG
  Wire.write(0b00101010); //OR, Interrupt on high events
  Wire.endTransmission();
  
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x32); //INT1_THS
  Wire.write(0b00001111); //Interrupt 1 Threshold
  Wire.endTransmission();
  
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x33); //INT1_DURATION
  Wire.write(0b00000011); //Interrupt 1 Duration
  Wire.endTransmission();
}

// INT1 will signal launch when acceleration exceeds threshold
void Deactivate_Lauch_Detection_LIS331HH() {
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x30); //INT1_CFG
  Wire.write(0b00000000); //Disabled
  Wire.endTransmission();
}

// INT2 will signal a zero-G state
void Activate_ZeroG_Detection_LIS331HH() {
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x20); //CTRL_REG1
  Wire.write(0b00100111); //On, 50Hz, axes enabled
  Wire.endTransmission();
  
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x34); //INT2_CFG
  Wire.write(0b10010101); //AND, Interrupt on low events
  Wire.endTransmission();
  
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x36); //INT2_THS
  Wire.write(0b00000001); //Interrupt 2 Threshold
  Wire.endTransmission();
  
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x37); //INT2_DURATION
  Wire.write(0b00000001); //Interrupt 2 Duration
  Wire.endTransmission();
}
