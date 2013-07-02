#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
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

// Watch Dog Timer Periods
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
#define WDT_16ms 0
#define WDT_32ms 1
#define WDT_64ms 2
#define WDT_128ms 3
#define WDT_250ms 4
#define WDT_500ms 5
#define WDT_1s 6
#define WDT_2s 7
#define WDT_4s 8
#define WDT_8s 9

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
uint16_t MPL3115A2_short = 0;
float MPL3115A2_aavg = 0;
float MPL3115A2_zeroed_aval = 0;
uint8_t MPL3115A2_afrac = 0;
uint16_t MPL3115A2_amax = 0;
uint8_t MPL3115A2_amaxfrac = 0;
uint16_t MPL3115A2_height_feet = 0;
byte LIS331HH_X_val = 0;
byte LIS331HH_Y_val = 0;
byte LIS331HH_Z_val = 0;
byte LIS331HH_X_low = 0;
byte LIS331HH_Y_low = 0;
byte LIS331HH_Z_low = 0;
uint8_t LIS331HH_max = 0;
uint8_t LIS331HH_peak = 0;
boolean MPL3115A2_Ready = 0;
boolean launchDetected = false;
boolean fastSample = true;
boolean buttonHold = false;
long elapsedTime = 0;
int myCounter = 0;

// EEPROM STUFF
#define EEPROM_START_ADDRESS_1 0
#define EEPROM_END_ADDRESS_1 750
#define EEPROM_START_ADDRESS_2 751
#define EEPROM_END_ADDRESS_2 1005
#define EEPROM_MAX_ALT_ADDRESS 1010
#define EEPROM_MAX_ACC_ADDRESS 1012
#define EEPROM_MAX_VEL_ADDRESS 1013
#define EEPROM_ZEROED_AVAL_ADDRESS 1015
int eeprom_address = 0;

// Structure for 3 bytes stored in EEPROM for max data
typedef struct {
  uint16_t altitude; // Feet
  byte acceleration; // g's (0.2g resolution)
} MAX_DATA;

MAX_DATA maxData;

void prepare_max_data() {
  maxData.altitude = MPL3115A2_height_feet;
  maxData.acceleration = LIS331HH_max;  
}

void max_to_EEPROM() {
  EEPROM.write(EEPROM_MAX_ALT_ADDRESS, maxData.altitude >> 8);
  EEPROM.write(EEPROM_MAX_ALT_ADDRESS+1, maxData.altitude);
  EEPROM.write(EEPROM_MAX_ACC_ADDRESS, maxData.acceleration);
}

void read_max_from_EEPROM() {
  maxData.altitude = EEPROM.read(EEPROM_MAX_ALT_ADDRESS);
  maxData.altitude = maxData.altitude << 8;
  maxData.altitude = maxData.altitude | EEPROM.read(EEPROM_MAX_ALT_ADDRESS+1);
  maxData.acceleration = EEPROM.read(EEPROM_MAX_ACC_ADDRESS);
}

// Structure for 3 bytes stored in EEPROM at every sample point
typedef struct {
  uint16_t altitude;
  byte acceleration;
} LOG_DATA;

LOG_DATA logData;

void prepare_log_data() {
  logData.altitude = MPL3115A2_short;
  logData.acceleration = LIS331HH_peak;  
}

void initialize_EEPROM(){
  eeprom_address = EEPROM_START_ADDRESS_1;
}

void log_to_EEPROM() {
  EEPROM.write(eeprom_address, logData.altitude >> 8);
  eeprom_address = eeprom_address + 1;
  EEPROM.write(eeprom_address, logData.altitude);
  eeprom_address = eeprom_address + 1;
  EEPROM.write(eeprom_address, logData.acceleration);
  eeprom_address = eeprom_address + 1;
}

void read_log_from_EEPROM(int start_address) {
  logData.altitude = EEPROM.read(start_address);
  logData.altitude = logData.altitude << 8;
  logData.altitude = logData.altitude | EEPROM.read(start_address+1);
  logData.acceleration = EEPROM.read(start_address+2);
}

// Function to read float from EEPROM
float eepromReadFloat(int address){
   union u_tag {
     byte b[4];
     float fval;
   } u;   
   u.b[0] = EEPROM.read(address);
   u.b[1] = EEPROM.read(address+1);
   u.b[2] = EEPROM.read(address+2);
   u.b[3] = EEPROM.read(address+3);
   return u.fval;
}

// Function to write float to EEPROM
void eepromWriteFloat(int address, float value){
   union u_tag {
     byte b[4];
     float fval;
   } u;
   u.fval=value;
 
   EEPROM.write(address  , u.b[0]);
   EEPROM.write(address+1, u.b[1]);
   EEPROM.write(address+2, u.b[2]);
   EEPROM.write(address+3, u.b[3]);
}
 
void setup(){
  cbi(ADCSRA,ADEN);                    // switch Analog to Digital converter OFF
  cbi(PRR,PRTIM1);
  cbi(PRR,PRTIM0);
  cbi(PRR,PRADC);
  
  power_adc_disable();
  power_spi_disable();
  power_usart0_disable();
  
  pinMode(USB_D_PLUS, INPUT); 
  digitalWrite(USB_D_PLUS, HIGH); // Turn on pull-up
  pinMode(USB_D_MINUS, INPUT); 
  pinMode(REDLED,INPUT);
  pinMode(GREENLED,INPUT);
  pinMode(BLUELED,INPUT);
  pinMode(BUTTON, INPUT);
  pinMode(ALT_INT1, INPUT);
  //digitalWrite(ALT_INT1, HIGH);
  pinMode(ALT_INT2, INPUT);
  //digitalWrite(ALT_INT2, HIGH);
  pinMode(ACC_INT1, INPUT);
  pinMode(ACC_INT2, INPUT);
  digitalWrite(BUTTON, HIGH); // Turn on pull-up
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
      power_twi_enable();
      Sleep_LIS331HH();
      Sleep_MPL3115A2();
      power_twi_disable();
      pinMode(USB_D_PLUS, INPUT);
      pinMode(USB_D_MINUS, INPUT); 
      digitalWrite(USB_D_PLUS, HIGH);
      EIMSK |= _BV(INT0);
      read_max_from_EEPROM();
      sei(); // Enable interrups
      Red_LED_On(1);
      delay(10);
      Red_LED_On(0);
      Green_LED_On(1);
      delay(10);
      Green_LED_On(0);
      Blue_LED_On(1);
      delay(10);
      Blue_LED_On(0);
      Enable_LEDs(0);
      system_sleep(); // Go to sleep
      break;
    case 1: //Average 16 samples for zeroed value
      power_twi_enable();
      delay_sleep(WDT_1s);     
      Red_LED_On(1);
      Average_Alt_Data_MPL3115A2(16);
      MPL3115A2_zeroed_aval = MPL3115A2_aavg;
      eepromWriteFloat(EEPROM_ZEROED_AVAL_ADDRESS, MPL3115A2_zeroed_aval);
      Clear_Max_Alt_MPL3115A2();
      Reset_Max_Acc_LIS311HH();
      Red_LED_On(0);
      Activate_Lauch_Detection_LIS331HH();
      initialize_EEPROM();
      system_sleep();
      break;
    case 2: //Take Altitude and Acceleration Measurements with WDT (64ms)
      if (launchDetected){
        Deactivate_Lauch_Detection_LIS331HH();
        launchDetected = false;
        Activate_ZeroG_Detection_LIS331HH();
        Measure_Alt_MPL3115A2_8x();
      }
      if (eeprom_address > EEPROM_END_ADDRESS_1) {
        state_change();
        system_sleep();
      }
      else
      if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
        f_wdt=0;
        Green_LED_On(1);
        //Measure_Alt_MPL3115A2_8x();
        while (!Data_Ready_MPL3115A2()){}
        Read_Alt_Data_Short_MPL3115A2();
        Read_Acc_Data_LIS331HH();
        Get_Peak_Acc_LIS331HH();
        Get_Max_Acc_LIS331HH();
        prepare_log_data();
        log_to_EEPROM();
        myCounter = myCounter + 3;
        Measure_Alt_MPL3115A2_16x();
        Green_LED_On(0);
        system_sleep();
      }
      break;
    case 3: //Take Altitude and Acceleration Measurements with WDT (500ms)
      if (eeprom_address > EEPROM_END_ADDRESS_2) {
        state_change();
      }
      else
      if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
        f_wdt=0;
        Green_LED_On(1);
        Measure_Alt_MPL3115A2_32x();
        while (!Data_Ready_MPL3115A2()){}
        Read_Alt_Data_Short_MPL3115A2();
        Read_Acc_Data_LIS331HH();
        Get_Peak_Acc_LIS331HH();
        Get_Max_Acc_LIS331HH();
        prepare_log_data();
        log_to_EEPROM();
        Green_LED_On(0);
        system_sleep();
      }
      break;
    case 4: //Compute Max Height in feet
    Sleep_LIS331HH();
    Sleep_MPL3115A2();
    Red_LED_On(1);
    Read_Max_Alt_MPL3115A2();
    if (MPL3115A2_amax-1 > MPL3115A2_zeroed_aval)
      MPL3115A2_height_feet = (int)floor((((MPL3115A2_amax-1)+MPL3115A2_amaxfrac*0.0625)-MPL3115A2_zeroed_aval)*3.281);
    else MPL3115A2_height_feet = 0;
    prepare_max_data();
    max_to_EEPROM();
    Red_LED_On(0);
    delay(5);
    //noTone(10);
    //state = 0;
    system_sleep();
    break;
    case 5:
    elapsedTime = millis();
    while (digitalRead(BUTTON) == LOW) {}
    elapsedTime = millis() - elapsedTime;
    if (elapsedTime > 1000)
        state = 1;
    else  
    if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
      //f_wdt=0;
      if (maxData.altitude/100 > 0) {
        pinMode(REDLED, OUTPUT);
        for (int i=0; i < maxData.altitude/100; i++) {
          digitalWrite(REDLED,HIGH);  // let led blink
          delay(100);
          digitalWrite(REDLED,LOW);
          delay(250);
        }
        Red_LED_On(0);
        delay(500);
        pinMode(GREENLED, OUTPUT);
        for (int i=0; i < (maxData.altitude/10)%10; i++) {
          digitalWrite(GREENLED,HIGH);  // let led blink
          delay(100);
          digitalWrite(GREENLED,LOW);
          delay(250);
        }
        Green_LED_On(0);
        delay(500);
        pinMode(BLUELED, OUTPUT);
        for (int i=0; i < (maxData.altitude%100)%10; i++) {
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
        for (int i=0; i < (maxData.altitude/10); i++) {
          digitalWrite(GREENLED,HIGH);  // let led blink
          delay(100);
          digitalWrite(GREENLED,LOW);
          delay(250);
        }
        Green_LED_On(0);
        delay(500);
        pinMode(BLUELED, OUTPUT);
        for (int i=0; i < (maxData.altitude)%10; i++) {
          digitalWrite(BLUELED,HIGH);  // let led blink
          delay(100);
          digitalWrite(BLUELED,LOW);
          delay(250);
        }
        Blue_LED_On(0);
      }
      else
      if (maxData.altitude > 0) {
        pinMode(BLUELED,OUTPUT);
        for (int i=0; i < maxData.altitude; i++) {
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
      digitalWrite(USB_D_PLUS, LOW);
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
      UsbKeyboard.typeCharacters("Max Height");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      read_max_from_EEPROM();
      UsbKeyboard.typeNumber(maxData.altitude);
      UsbKeyboard.typeCharacters("f");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeCharacters("Max Acc");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeFloat((maxData.acceleration/128.0)*24);
      UsbKeyboard.typeCharacters("g");
      UsbKeyboard.sendKeyStroke(KEY_ENTER);
      MPL3115A2_zeroed_aval = eepromReadFloat(EEPROM_ZEROED_AVAL_ADDRESS);
      //UsbKeyboard.typeCharacters("Alt Zeroed");
      //UsbKeyboard.sendKeyStroke(KEY_SPACE);
      //UsbKeyboard.typeFloat(MPL3115A2_zeroed_aval);
      //UsbKeyboard.sendKeyStroke(KEY_ENTER);
      UsbKeyboard.typeCharacters("Height(f) Accel(g)");
      UsbKeyboard.sendKeyStroke(KEY_ENTER);
      // Type out all logged points
      for (int i = 0; i < 1008; i=i+3) {
        read_log_from_EEPROM(i);
        //UsbKeyboard.typeNumber(logData.altitude>>2);
        //UsbKeyboard.sendKeyStroke(KEY_SPACE);
        //UsbKeyboard.typeNumber(logData.altitude&0x0003);
        //UsbKeyboard.sendKeyStroke(KEY_SPACE);
        if (((logData.altitude>>2)-1) > (int)MPL3115A2_zeroed_aval)
          UsbKeyboard.typeNumber((int)(((((logData.altitude>>2)-1)+(logData.altitude&0x0003)*0.25)-MPL3115A2_zeroed_aval)*3.281));
        else UsbKeyboard.typeNumber(0);
        UsbKeyboard.sendKeyStroke(KEY_SPACE);
        UsbKeyboard.typeFloat((logData.acceleration/128.0)*24);
        UsbKeyboard.sendKeyStroke(KEY_ENTER);
      }
      /*
      UsbKeyboard.typeCharacters("AMAX");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeNumber(MPL3115A2_amax);
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      read_log_from_EEPROM(3);
      UsbKeyboard.typeCharacters("Address");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeNumber(3);
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeCharacters("Alt Zeroed");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeFloat(eepromReadFloat(EEPROM_ZEROED_AVAL_ADDRESS));
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeCharacters("Alt Meters");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeNumber(logData.altitude >> 2);
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeCharacters("Alt Feet");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeNumber(logData.altitude&0x0003);
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeCharacters("Height");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      if ((logData.altitude>>2) > eepromReadFloat(EEPROM_ZEROED_AVAL_ADDRESS))
        UsbKeyboard.typeNumber((((logData.altitude>>2)-MPL3115A2_zeroed_aval)*3)+(logData.altitude&0x0003));
      else UsbKeyboard.typeNumber(0);
      UsbKeyboard.typeCharacters("f");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeCharacters("Acceleration");
      UsbKeyboard.sendKeyStroke(KEY_SPACE);
      UsbKeyboard.typeFloat((logData.acceleration/128.0)*24);
      UsbKeyboard.typeCharacters("g");
      UsbKeyboard.sendKeyStroke(KEY_ENTER);
      
      */
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
  MCUCR = _BV (BODS) | _BV (BODSE);  // turn on brown-out enable select
  MCUCR = _BV (BODS);        // this must be done within 4 clock cycles of above
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
}

// sleep for one watchdog timer period
void delay_sleep(int timeout){
  setup_watchdog(timeout);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  cli();
  sleep_enable();
  //sleep_bod_disable();
  sei();
  MCUCR = _BV (BODS) | _BV (BODSE);  // turn on brown-out enable select
  MCUCR = _BV (BODS);        // this must be done within 4 clock cycles of above
  sleep_cpu();              // sleep within 3 clock cycles of above
  sleep_disable();                     // System continues execution here when watchdog timed out
  wdt_disable(); 
  sei();
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
      state = 5;
      wdt_disable();
      f_wdt=1;
      break;
    case 1: //zero height
      state = 2;
      setup_watchdog(WDT_64ms);
      launchDetected = true;
      break;
    case 2: //fast samples
      state = 3;
      setup_watchdog(WDT_500ms);
      break;
    case 3: //slow samples
      state = 4;
      wdt_disable();
      break;
    case 4: //stop aquiring height
      f_wdt=1;
      state = 5;
      wdt_disable();
      break;
    case 5: //flash out height
      f_wdt=1;
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
      setup_watchdog(WDT_64ms);
      launchDetected = true;
      break;
    case 2: //fast samples
      break;
    case 3: //slow samples
      break;
    case 4: //stop aquiring height
      break;
    case 5: //flash out height
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
    case 2: //fast samples
      state = 3;
      setup_watchdog(WDT_500ms);
      break;
    case 3: //slow samples
      break;
    case 4: //stop aquiring height
      break;
    case 5: //flash out height
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
  Wire.write(0x22); //Active high and push-pull
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

// 8x OS takes min 34ms
void Measure_Alt_MPL3115A2_8x () {
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x26); //CTRL_REG1
  Wire.write(0b10011010); //Alt, 8x OS, One-Shot, Standby
  Wire.endTransmission();
}

// 16x os takes min 66ms
void Measure_Alt_MPL3115A2_16x () {
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x26); //CTRL_REG1
  Wire.write(0b10100010); //Alt, 16x OS, One-Shot, Standby
  Wire.endTransmission();
}

// 32x OS takes min 130ms
void Measure_Alt_MPL3115A2_32x () {
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x26); //CTRL_REG1
  Wire.write(0b10101010); //Alt, 32x OS, One-Shot, Standby
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

void Read_Alt_Data_Full_MPL3115A2() {
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

// 14 MSBs for height in meters and 2 LSBs for 0.25 meters
// Estimate feet by taking meters x 3 and add fractional bits as feet
void Read_Alt_Data_Short_MPL3115A2() {
  Wire.beginTransmission(MPL3115A2_ADDRESS);
  Wire.write(0x01); //Pressure/Alt Data Out MSB - auto increments
  Wire.endTransmission(false); //Keep bus active
  Wire.requestFrom(MPL3115A2_ADDRESS, 3); // request 3 bytes
  while(Wire.available()){  // slave may send less than requested
    MPL3115A2_aval_msb = Wire.read();  
    MPL3115A2_aval_csb = Wire.read();
    MPL3115A2_aval_lsb = Wire.read();
  }
  MPL3115A2_short = MPL3115A2_aval_msb;
  MPL3115A2_short = MPL3115A2_short << 10;
  MPL3115A2_short = MPL3115A2_short >> 2;
  MPL3115A2_short = MPL3115A2_short | MPL3115A2_aval_csb;
  MPL3115A2_short = MPL3115A2_short << 2;
  //MPL3115A2_aval_lsb = MPL3115A2_aval_lsb >> 6;
  MPL3115A2_short = MPL3115A2_short | (MPL3115A2_aval_lsb >> 6);
}

void Average_Alt_Data_MPL3115A2 (int samples) {
  for (int i=0; i<samples; i++){
    Measure_Alt_MPL3115A2_32x();
    while (!Data_Ready_MPL3115A2()){}
    //while(digitalRead(ALT_INT1) == HIGH){} //Wait until INT1 goes low
    Read_Alt_Data_Full_MPL3115A2();
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
  Low_Rate_LIS331HH();
  
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

void High_Rate_LIS331HH() {
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x20); //CTRL_REG1
  Wire.write(0b00100111); //On, 50Hz, axes enabled
  Wire.endTransmission();
}

void Low_Rate_LIS331HH() {
  Wire.beginTransmission(LIS331HH_ADDRESS);
  Wire.write(0x20); //CTRL_REG1
  Wire.write(0b11000111); //Low Power 10Hz, axes enabled
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
  
  High_Rate_LIS331HH();
  
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
  
  High_Rate_LIS331HH();
  
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

// Read out acceleration data
void Read_Acc_Data_LIS331HH() {
  for (int i=0x28; i < 0x2E; i++) {
    Wire.beginTransmission(LIS331HH_ADDRESS);
    Wire.write(i);
    Wire.endTransmission();
    Wire.requestFrom(LIS331HH_ADDRESS, 1);    // request 1 byte from slave device
    while(Wire.available()){    // slave may send less than requested
      switch (i) {
        case 0x28: 
          LIS331HH_X_low = Wire.read(); // X LSBs
          break;
        case 0x29:
          LIS331HH_X_val = Wire.read(); // X MSBs
          break;
        case 0x2A:
          LIS331HH_Y_low = Wire.read(); // Y LSBs
          break;
        case 0x2B:
          LIS331HH_Y_val = Wire.read(); // Y MSBs
          break;
        case 0x2C:
          LIS331HH_Z_low = Wire.read(); // Z LSBs
          break;
        case 0x2D:
          LIS331HH_Z_val = Wire.read(); // Z MSBs
          break;
      }
    }
  }
}
void Reset_Max_Acc_LIS311HH() {
  LIS331HH_max = 0;
}

//Max value across axes for all samples
void Get_Max_Acc_LIS331HH() {
  // Compute absolute values
  if (LIS331HH_X_val > 127)
    LIS331HH_X_val = 256-LIS331HH_X_val;
  if (LIS331HH_Y_val > 127)
    LIS331HH_Y_val = 256-LIS331HH_Y_val;
  if (LIS331HH_Z_val > 127)
    LIS331HH_Z_val = 256-LIS331HH_Z_val;
  // Determine which axis is max
  if (LIS331HH_max < LIS331HH_X_val)
    LIS331HH_max = LIS331HH_X_val;
  if (LIS331HH_max < LIS331HH_Y_val)
    LIS331HH_max = LIS331HH_Y_val;
  if (LIS331HH_max < LIS331HH_Z_val)
    LIS331HH_max = LIS331HH_Z_val;
}

// Peak value across axes for one sample
void Get_Peak_Acc_LIS331HH() {
  // Compute absolute values
  if (LIS331HH_X_val > 127)
    LIS331HH_X_val = 256-LIS331HH_X_val;
  if (LIS331HH_Y_val > 127)
    LIS331HH_Y_val = 256-LIS331HH_Y_val;
  if (LIS331HH_Z_val > 127)
    LIS331HH_Z_val = 256-LIS331HH_Z_val;
  // Determine which axis is highest
  LIS331HH_peak = LIS331HH_X_val;
  if (LIS331HH_peak < LIS331HH_Y_val)
    LIS331HH_peak = LIS331HH_Y_val;
  if (LIS331HH_peak < LIS331HH_Z_val)
    LIS331HH_peak = LIS331HH_Z_val;
}
