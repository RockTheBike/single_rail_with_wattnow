/**** Split-rail Pedalometer
 * Arduino code to run the sLEDgehammer
 * ver. 2.0
 * Written by:
 * Thomas Spellman <thomas@thosmos.com>
 * Jake <jake@spaz.org>
 * Paul@rockthebike.com
 * Copyright: 2012, Rock the Bike (http://rockthebike.com)
 * License: This code is distributed under the GPL license: http://www.gnu.org/licenses/gpl.html
 * 
 * 1.6 -- First version with Wattage output display. 
 * 1.7 -- cleaned up serial output, six LED pedalometer, removed whichPin[], inverted minusRelay state (minus relay is wired normally-open now), re-inits display every minute [jake]
 * 1.8 -- added minusAlert (makes all lights blink and displays MORE KIDS if minus is less than 14 volts), set all text to Orange, only updates display once per mode-change [jake]
 * 1.9 -- added support for wattnow sign, changed serial rate to 9600 [jake]
 * 2.0 -- Brought over code from Pikes Peak that makes Minus Alert only display if Minus is involved. 
 * 2.1x -- modified for Fariche customer (no minusrail, single ampmeter/wattmeter) Arbduino board
 */

#define BAUDRATE 57600

const char versionStr[] = "Pedal Power Utility Box ver. 2.1x . Single_Rail_with_WattNow_Fariche.pde";

#include <Wire.h>
#include <avr/pgmspace.h>
#include "ht1632c.h"
#include "font1.h"

//this new pinout corresponds to the first two-team maker-faire arbduino setup
//         arduino pin number v       display pin # v
static const byte ht1632_cs = 7;    // Chip Select (pin 1)
static const byte ht1632_clk = 4;  // Clk pin (pin 2)
static const byte ht1632_wrclk = 12; // Write clock pin (pin 5)
static const byte ht1632_data = 13;  // Data pin (pin 7)

#define DISPLAYRATE 2000 // how many milliseconds to show each text display before switching

float wattAvg,wattAvgAdder = 0;  // used for averaging wattage for wattnow sign
#define WATTAVGCOUNT 10 // how many main loops to average wattAvgAdder into wattAvg
int wattAvgIndex = 0;  // how many times we have counted wattage

float wattHours = 0;  // how many watt-hours since bootup
unsigned long lastWattHours = 0; // the last time we measured watt-hours

#define pwm 0
#define onoff 1

// Arduino pin for each output level
const int numPins = 4; // Number of active Arduino Pins for + rail team.
int pin[numPins] = {
  3, 5, 6, 9}; 

#define numLevels numPins + 2
// voltages at which to turn on each level
// voltage above the second to last level makes the last LEDs fastblink
// voltage above the last level makes ALL the LEDs fastblink
float levelVolt[numLevels] = {
  22.0, 23.5, 24.8, 26.6, 27.0, 28.0};

// voltages at which to turn on each level
// float levelVolt[numLevels] = {21, 24.0, 27.0};

int levelType[numLevels] = {
  pwm, pwm, pwm, pwm, pwm};

// type of each level (pwm or onoff)

// Arduino pins to be used as inputs to voltage sensor. This shows where to connect the wires! 
const int voltPin = A0; // Voltage Sensor Input
const int ampsPin = A3; // AC power and all DC devices. 
const int relayPin=2;
#define VOLTCOEFF 13.1944  // 304 / 23.04v
#define AMPCOEFF 8.0682 // 583 - 512 = 71; 71 / 8.8 amps = 8.0682
#define AMPOFFSET 512 // when current sensor is at 0 amps this is the ADC value

//MAXIMUM VOLTAGE TO GIVE LEDS
//const float maxVoltLEDs = 24 -- always run LEDs in 24V series configuration.
const float maxVoltLEDs = 10.0; //LED
const float maxVolt = 27.4;

//Hysteresis variables
const float ComeBackInVoltage = 26;
int Hysteresis=0;

// vars to store temp values
int adcvalue = 0;

//Voltage related variables. 
float voltage = 0;

//Current related variables
float amps=0;
int ampsRaw;

float Watts;
float wattage;
// on/off state of each level (for use in status output)
int desiredState[numPins];

#define AVG_CYCLES 7 // average measured voltage over this many samples
#define DISPLAY_INTERVAL_MS 1000 // when auto-display is on, display every this many milli-seconds
#define RESETINTERVAL 60 // how many display intervals between display resets 
int displayCount = 0;  // counts how many display intervals have happened
int displaymode = 0; // stores what display mode we're on (W.H. or WATT or whatever)
char buf[]="    "; // stores the number we're going to display

int readCount = 0; // for determining how many sample cycle occur per display interval

// vars for current PWM duty cycle
int pwmValue;
boolean updatePwm = false;


// timing variables for blink. Mark, there are too many of these. 
int blinkState=0;
int fastBlinkState=0;
unsigned long time = 0;
unsigned long lastFastBlinkTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long timeRead = 0;
unsigned long timeDisplay = 0;
unsigned long timeWeStartedCheckingForVictory = 0;

// Time related variables. Mark, this may be a vestige of the sLEDgehammer code but we should bring this feature back because in this
//video, the flickering would be prevented:
//Basically this is a minimum time to keep each level on for before you can change it. http://www.rockthebike.com/pedal-power-utility
//box-from-rock-the-bike/  
unsigned long onTime = 500;
unsigned long levelTime = 0;

// current display level
//int level = -1;

// var for looping through arrays
int i = 0;
int x = 0;
int y = 0;


//About to add a bunch of variables related to the ht1632 display. 

//#include <Wire.h>
//#include <avr/pgmspace.h>
//
//#include "ht1632c.h"
//#include "font1.h"

// Dual 3216 LED Matrix
// If you have only one set these to:
// X_MAX=31
// Y_MAX=15
// CHIP_MAX=4
#define X_MAX 63 // 0 based X
#define Y_MAX 15 // 0 based Y
#define CHIP_MAX 4*2 // Number of HT1632C Chips
// 4 each board * 2 boards

// possible values for a pixel;
#define BLACK  0
#define GREEN  1
#define RED    2
#define ORANGE 3

#define ASSERT(condition) //nothing

/*
 * Set these constants to the values of the pins connected to the SureElectronics Module
 */
// The should also be a common GND (pins .
// The module with all LEDs like draws about 200mA,
//  which makes it PROBABLY powerable via Arduino +5V

/*
 * we keep a copy of the display controller contents so that we can
 * know which bits are on without having to (slowly) read the device.
 * Note that we only use the low four bits of the shadow ram, since
 * we're shadowing 4-bit memory.  This makes things faster, but we
 * COULD do something with the other half of our bytes !
 */
byte ht1632_shadowram[63][CHIP_MAX] = {
  0};
/*
 * we use buffer to put the char fonts
 */
char buffer[8][8];

//**************************************************************************************************
//Function Name: decToBcd
//Function Feature: Convert normal decimal numbers to binary coded decimal
//Input Argument: void
//Output Argument: void
//**************************************************************************************************



void setup() {
  Serial.begin(BAUDRATE);
  ht1632_initialize();

  // Initialize Software PWM
  //SoftPWMBegin();

  Serial.println(versionStr);

  pinMode(relayPin, OUTPUT);

  // init LED pins
  for(i = 0; i < numPins; i++) {
    pinMode(pin[i],OUTPUT);
    if(levelType[i] == pwm)
      analogWrite(pin[i],0);
    else if(levelType[i] == onoff)
      digitalWrite(pin[i],0);
  }

  getVoltages();

}

boolean levelLock = false;
int senseLevel = -1;

void loop() {

  time = millis();
  getVoltages();

  wattAvgAdder += voltage * amps;  // add instantaneous wattage to wattAvgAdder
  if (++wattAvgIndex >= WATTAVGCOUNT) {
    wattAvg = wattAvgAdder / WATTAVGCOUNT; // set wattAvg to the average value
    wattAvgAdder = 0; // start adder over
    wattAvgIndex = 0; // start the averaging over
  }  // this is how we made wattAvg

  wattHours += wattAvg * ((float) (time - lastWattHours)/3600000);  // milliseconds to hours conversion
  lastWattHours = time;

  updateDisplay();  // update the ht1632c display
  setpwmvalue();
  readCount++;
  //First deal with the blink  

  if (((time - lastBlinkTime) > 600) && blinkState==1){
    //                  Serial.println("I just turned pwm to 0.");
    //     pwmValue=0;
    blinkState=0;
    //   analogWrite(pin[i], pwmValue);
    lastBlinkTime=time;
  } 
  else if (((time - lastBlinkTime) > 600) && blinkState==0){
    //   Serial.println("I just turned blinkstate to 1.");
    blinkState=1; 
    lastBlinkTime=time;
  }


  if (((time - lastFastBlinkTime) > 120) && fastBlinkState==1){
    //                  Serial.println("I just turned pwm to 0.");
    //     pwmValue=0;
    fastBlinkState=0;
    //   analogWrite(pin[i], pwmValue);
    lastFastBlinkTime=time;
  } 
  else if (((time - lastFastBlinkTime) > 120) && fastBlinkState==0){
    //   Serial.println("I just turned blinkstate to 1.");
    fastBlinkState=1; 
    lastFastBlinkTime=time;
  }  

  //protect the ultracapacitors if needed
  if (voltage > maxVolt){
    digitalWrite(relayPin, HIGH); 
    Hysteresis=1;
  }


  if (Hysteresis==1 && voltage < ComeBackInVoltage){
    digitalWrite(relayPin, LOW);
    Hysteresis=0;
  } 

  //Set the desired lighting states. 

  senseLevel = -1;
  if (voltage <=levelVolt[0]){  // voltage is below minimum
    senseLevel=0;
    desiredState[0]=1; // set first lights to slowblink
  } 
  else {  // set lights programmatically with for() loop

    for(i = 0; i < numLevels; i++) {  // what to do if voltage points are met

      if(voltage >= levelVolt[i]){
        senseLevel = i;
        if (i > 0) desiredState[0] = 0; // if second level, turn off first (red) lights
        if (i < numPins) {  // if its an actual LED level
          desiredState[i]=2; // turn lights solid on
        }
        else { // otherwise it's the overvoltage level so
          desiredState[numPins-1] = 3; // set top LEDs to fastblink
          if (i > numPins) { // relays must have failed shut!!!  freak out!
            for (int ohshit = 0; ohshit < numPins; ohshit++) desiredState[ohshit] = 3;
          } // we just had to set ALL LEDs to fastblink
        }
      } 
      else {
        if (i < numPins) desiredState[i]=0;  // turn the level off
      }
    }
  }
//
//  if (senseLevel == numPins+1){
//    desiredState[numPins] = 3; // workaround to blink white lights
//  }

  // End setting desired states. 
  // Do the desired states. 
  // loop through each led and turn on/off or adjust PWM

  for(i = 0; i < numPins; i++) {

    if(levelType[i]==pwm) {

      if(desiredState[i]==2){
        analogWrite(pin[i], pwmValue);

      }
      else if (desiredState[i]==0){
        analogWrite(pin[i], 0);
      }
      else if (desiredState[i]==1 && blinkState==1){
        analogWrite(pin[i], pwmValue);
      }
      else if (desiredState[i]==1 && blinkState==0){
        analogWrite(pin[i], 0);
      }
      else if (desiredState[i]==3 && fastBlinkState==1){
        analogWrite(pin[i], pwmValue);
      }
      else if (desiredState[i]==3 && fastBlinkState==0){
        analogWrite(pin[i], 0);

      }
    } 
  } //end for

  delay(10);
}

void updateDisplay() {

  if(time - timeDisplay > DISPLAY_INTERVAL_MS){
    timeDisplay = time;
    displayCount += 1; //increment displayCount counter
    if (displayCount > RESETINTERVAL) {
      displayCount = 0;
      ht1632_initialize();
    }
    printDisplay(); // print stuff to serial port about everything
    readCount = 0;
  }

  char* label;
  if( time % (DISPLAYRATE * 2) > DISPLAYRATE ) {  // display wattage
    if (displaymode != 1) {
      sprintf(buf, "%4d", (int) wattAvg);  // only calculate wattage once per display update
    }
    displaymode = 1;
    label = "WATT";
  } 
  else {  // display WATT HOURS
    if (displaymode != 2) {
      sprintf(buf, "%4d", (int) (wattHours));
    }
    label = "W.H. ";
    displaymode = 2;
  } 

  //sprintf(buf, "%4.1f", voltage);
  ht1632_draw_strings( label, buf );

}

void setpwmvalue()
{

  // The effective voltage of a square wave is
  // Veff = Vin * sqrt(k)
  // where k = t/T is the duty cycle

    // If you want to make 12V from a 20V input you have to use
  // k = (Veff/Vin)^2 = (12/20)^2 = 0.36

  int newVal = 0;

  if (voltage <= maxVoltLEDs) {
    newVal = 255.0;
  }
  else {
    newVal = maxVoltLEDs / voltage * 255.0;
  }

  // if(voltage <= 24) {
  // pwmvalue24V = 255.0;
  // }
  // else {
  // pwmvalue24V = sq(24 / voltage) * 255.0;
  // }

  if(newVal != pwmValue) {
    pwmValue = newVal;
    updatePwm = true;
  }
}

void getVoltages(){

  voltage = average((analogRead(voltPin) / VOLTCOEFF), voltage); //Felt slow so I am putting in the line below with no averaging

  amps = (float) ((analogRead(ampsPin) - AMPOFFSET) / AMPCOEFF);
}

float average(float val, float avg){
  if(avg == 0)
    avg = val;
  return (val + (avg * (AVG_CYCLES - 1))) / AVG_CYCLES;
}

void printDisplay(){
  Serial.print(" volts: ");
  Serial.print(voltage);
  Serial.print(" (");
  Serial.print(analogRead(voltPin));
  if (Hysteresis) Serial.print(" PROTECTED ");
  Serial.print("), amps: ");
  Serial.print(amps);
  Serial.print(" (");
  Serial.print(analogRead(ampsPin));
  Serial.print("), WattHours: ");
  Serial.print(wattHours);
  Serial.print(", WattAvg: ");
  Serial.print(wattAvg);
  //  Serial.print(", Wattage: ");
  //  Serial.print(voltage * amps);
  Serial.print(", pwm value: ");
  Serial.print(pwmValue);

  Serial.print(", LEDLEVEL:MODE ");
  for(i = 0; i < numPins; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(desiredState[i]);
    Serial.print(",");
  }

  Serial.println();

  //  
  //    Serial.print("Desired State ");
  //  
  //  for(i = 0; i < numLevels; i++) {
  //    Serial.print(i);
  //    Serial.print(": ");
  //    Serial.print(desiredState[i]);
  //    Serial.print(", ");
  //  }
  //  Serial.println();

}


byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

//**************************************************************************************************
//Function Name: bcdToDec
//Function Feature: Convert binary coded decimal to normal decimal numbers
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

//**************************************************************************************************
//Function Name: OutputCLK_Pulse
//Function Feature: enable CLK_74164 pin to output a clock pulse
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
void OutputCLK_Pulse(void)	//Output a clock pulse
{
  digitalWrite(ht1632_clk, HIGH);
  digitalWrite(ht1632_clk, LOW);
}

//**************************************************************************************************
//Function Name: ht1632_chipselect
//Function Feature: enable HT1632C
//Input Argument: select: HT1632C to be selected
//	If select=0, select none.
//	If s<0, select all.
//Output Argument: void
//**************************************************************************************************
void ht1632_chipselect(int select)
{
  unsigned char tmp = 0;
  if (select < 0) { // Enable all HT1632C
    digitalWrite(ht1632_cs, LOW);
    for (tmp=0; tmp<CHIP_MAX; tmp++) {
      OutputCLK_Pulse();
    }
  } 
  else if(select==0) { //Disable all HT1632Cs
    digitalWrite(ht1632_cs, HIGH);
    for(tmp=0; tmp<CHIP_MAX; tmp++) {
      OutputCLK_Pulse();
    }
  } 
  else {
    digitalWrite(ht1632_cs, HIGH);
    for(tmp=0; tmp<CHIP_MAX; tmp++) {
      OutputCLK_Pulse();
    }
    digitalWrite(ht1632_cs, LOW);
    OutputCLK_Pulse();
    digitalWrite(ht1632_cs, HIGH);
    tmp = 1;
    for( ; tmp<select; tmp++) {
      OutputCLK_Pulse();
    }
  }
}

//**************************************************************************************************
//Function Name: ht1632_writebits
//Function Feature: Write bits (up to 8) to h1632 on pins ht1632_data, ht1632_wrclk
//                  Chip is assumed to already be chip-selected
//                  Bits are shifted out from MSB to LSB, with the first bit sent
//                  being (bits & firstbit), shifted till firsbit is zero.
//Input Argument: bits: bits to send
//	      firstbit: the first bit to send
//Output Argument: void
//**************************************************************************************************
void ht1632_writebits (byte bits, byte firstbit)
{
  DEBUGPRINT(" ");
  while (firstbit) {
    DEBUGPRINT((bits&firstbit ? "1" : "0"));
    digitalWrite(ht1632_wrclk, LOW);
    if (bits & firstbit) {
      digitalWrite(ht1632_data, HIGH);
    }
    else {
      digitalWrite(ht1632_data, LOW);
    }
    digitalWrite(ht1632_wrclk, HIGH);
    firstbit >>= 1;
  }
}

//**************************************************************************************************
//Function Name: ht1632_sendcmd
//Function Feature: Send a command to the ht1632 chip.
//                  Select 1 0 0 c7 c6 c5 c4 c3 c2 c1 c0 xx Free
//Input Argument: chipNo: the chip you want to send data
//               command: consists of a 3-bit "CMD" ID, an 8bit command, and
//                        one "don't care bit".
//Output Argument: void
//**************************************************************************************************
static void ht1632_sendcmd (byte chipNo, byte command)
{
  ht1632_chipselect(chipNo);
  ht1632_writebits(HT1632_ID_CMD, 1<<2);  // send 3 bits of id: COMMMAND
  ht1632_writebits(command, 1<<7);  // send the actual command
  ht1632_writebits(0, 1); 	/* one extra dont-care bit in commands. */
  ht1632_chipselect(0);
}

//**************************************************************************************************
//Function Name: ht1632_senddata
//Function Feature: send a nibble (4 bits) of data to a particular memory location of the
//                  ht1632.  The command has 3 bit ID, 7 bits of address, and 4 bits of data.
//                  Select 1 0 1 A6 A5 A4 A3 A2 A1 A0 D0 D1 D2 D3 Free
//                  Note that the address is sent MSB first, while the data is sent LSB first!
//                  This means that somewhere a bit reversal will have to be done to get
//                  zero-based addressing of words and dots within words.
//Input Argument: chipNo: the chip you want to send data
//               address: chip address to write
//                  data: data to write to chip memory
//Output Argument: void
//**************************************************************************************************
static void ht1632_senddata (byte chipNo, byte address, byte data)
{
  ht1632_chipselect(chipNo);
  ht1632_writebits(HT1632_ID_WR, 1<<2);  // send ID: WRITE to RAM
  ht1632_writebits(address, 1<<6); // Send address
  ht1632_writebits(data, 1<<3); // send 4 bits of data
  ht1632_chipselect(0);
}

//**************************************************************************************************
//Function Name: ht1632_clear
//Function Feature: clear display
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
void ht1632_clear()
{
  char i;

  for (int j=1;j<=CHIP_MAX;j++) {
    ht1632_chipselect(j);
    ht1632_writebits(HT1632_ID_WR, 1<<2);  // send ID: WRITE to RAM
    ht1632_writebits(0, 1<<6); // Send address
    for (i=0; i<64/2; i++) // Clear entire display
      ht1632_writebits(0, 1<<7); // send 8 bits of data
    ht1632_chipselect(0);
    for (i=0; i<64; i++)
      ht1632_shadowram[i][j] = 0;
  }
}

//**************************************************************************************************
//Function Name: xyToIndex
//Function Feature: get the value of x,y
//Input Argument: x: X coordinate
//                y: Y coordinate
//Output Argument: address of xy
//**************************************************************************************************
byte xyToIndex(byte x, byte y)
{
  byte nChip, addr;

  if (x>=32) {
    nChip = 3 + x/16 + (y>7?2:0);
  } 
  else {
    nChip = 1 + x/16 + (y>7?2:0);
  }

  x = x % 16;
  y = y % 8;
  addr = (x<<1) + (y>>2);

  return addr;
}

//**************************************************************************************************
//Function Name: calcBit
//Function Feature: calculate the bitval of y
//Input Argument: y: Y coordinate
//Output Argument: bitval
//**************************************************************************************************
#define calcBit(y) (8>>(y&3))

//**************************************************************************************************
//Function Name: get_pixel
//Function Feature: get the value of x,y
//Input Argument: x: X coordinate
//                y: Y coordinate
//Output Argument: color setted on x,y coordinates
//**************************************************************************************************
int get_pixel(byte x, byte y) {
  byte addr, bitval, nChip;

  if (x>=32) {
    nChip = 3 + x/16 + (y>7?2:0);
  } 
  else {
    nChip = 1 + x/16 + (y>7?2:0);
  }

  addr = xyToIndex(x,y);
  bitval = calcBit(y);

  if ((ht1632_shadowram[addr][nChip-1] & bitval) && (ht1632_shadowram[addr+32][nChip-1] & bitval)) {
    return ORANGE;
  } 
  else if (ht1632_shadowram[addr][nChip-1] & bitval) {
    return GREEN;
  } 
  else if (ht1632_shadowram[addr+32][nChip-1] & bitval) {
    return RED;
  } 
  else {
    return 0;
  }
}

//**************************************************************************************************
//Function Name: ht1632_plot
//Function Feature: plot a dot on x,y
//Input Argument: x: X coordinate
//                y: Y coordinate
//            color: BLACK(clean), GREEN, RED, ORANGE
//Output Argument: void
//**************************************************************************************************
void ht1632_plot (byte x, byte y, byte color)
{
  byte nChip, addr, bitval;

  if (x<0 || x>X_MAX || y<0 || y>Y_MAX)
    return;

  if (color != BLACK && color != GREEN && color != RED && color != ORANGE)
    return;

  if (x>=32) {
    nChip = 3 + x/16 + (y>7?2:0);
  } 
  else {
    nChip = 1 + x/16 + (y>7?2:0);
  }

  addr = xyToIndex(x,y);
  bitval = calcBit(y);

  switch (color)
  {
  case BLACK:
    if (get_pixel(x,y) != BLACK) { // compare with memory to only set if pixel is other color
      // clear the bit in both planes;
      ht1632_shadowram[addr][nChip-1] &= ~bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
      addr = addr + 32;
      ht1632_shadowram[addr][nChip-1] &= ~bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
    }
    break;
  case GREEN:
    if (get_pixel(x,y) != GREEN) { // compare with memory to only set if pixel is other color
      // set the bit in the green plane and clear the bit in the red plane;
      ht1632_shadowram[addr][nChip-1] |= bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
      addr = addr + 32;
      ht1632_shadowram[addr][nChip-1] &= ~bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
    }
    break;
  case RED:
    if (get_pixel(x,y) != RED) { // compare with memory to only set if pixel is other color
      // clear the bit in green plane and set the bit in the red plane;
      ht1632_shadowram[addr][nChip-1] &= ~bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
      addr = addr + 32;
      ht1632_shadowram[addr][nChip-1] |= bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
    }
    break;
  case ORANGE:
    if (get_pixel(x,y) != ORANGE) { // compare with memory to only set if pixel is other color
      // set the bit in both the green and red planes;
      ht1632_shadowram[addr][nChip-1] |= bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
      addr = addr + 32;
      ht1632_shadowram[addr][nChip-1] |= bitval;
      ht1632_senddata(nChip, addr, ht1632_shadowram[addr][nChip-1]);
    }
    break;
  }
}

//**************************************************************************************************
//Function Name: set_buffer
//Function Feature: set buffer variable to char
//Input Argument: chr: char to set on buffer
//Output Argument: void
//**************************************************************************************************
void set_buffer(char chr){
  for(int i=0; i<sizeof(chl); i++){
    if(chl[i] == chr){
      int pos = i*8;
      for(int j=0;j<8;j++){
        memcpy_P(buffer[j], (PGM_P)pgm_read_word(&(CHL[j+pos])), 8);
      }
    }
  }
}

//**************************************************************************************************
//Function Name: null_buffer
//Function Feature: null entire buffer
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
void null_buffer(){
  for(int i=0;i<8;i++)
    for(int j=0; j<8;j++)
      buffer[i][j] = 0;
}

//**************************************************************************************************
//Function Name: ht1632_initialize
//Function Feature: initialize the display
//Input Argument: void
//Output Argument: void
//**************************************************************************************************
static void ht1632_initialize()
{
  pinMode(ht1632_cs, OUTPUT);
  digitalWrite(ht1632_cs, HIGH);
  pinMode(ht1632_wrclk, OUTPUT);
  pinMode(ht1632_data, OUTPUT);
  pinMode(ht1632_clk, OUTPUT);
  for (int i=1; i<=CHIP_MAX; i++) {
    ht1632_sendcmd(i, HT1632_CMD_SYSDIS); // Disable system
    ht1632_sendcmd(i, HT1632_CMD_COMS00); // 16*32, PMOS drivers
    ht1632_sendcmd(i, HT1632_CMD_MSTMD);  // MASTER MODE
    ht1632_sendcmd(i, HT1632_CMD_SYSON);  // System on
    ht1632_sendcmd(i, HT1632_CMD_LEDON);  // LEDs on 
  }
  ht1632_clear(); // Clear the display
}

void ht1632_draw_strings( char* STRING1, char* STRING2 )
{
  int x,i,j;
  for(x=1;x<5;x++) {
    null_buffer();
    for(i=0;i<8;i++){
      for(j=0;j<8;j++){
        set_buffer(STRING1[x-1]);
        if (~buffer[i][j] & (1<<0)) {
          ht1632_plot(j+(8*(x-1))-1,i,ORANGE);
        } 
        else {
          ht1632_plot(j+(8*(x-1))-1,i,BLACK);
        }
      }
    }
  }
  for(x=1;x<5;x++) {
    null_buffer();
    for(i=0;i<8;i++){
      for(j=0;j<8;j++){
        set_buffer(STRING2[x-1]);
        if (~buffer[i][j] & (1<<0)) {
          ht1632_plot(j+(8*(x-1))-1,i+8,ORANGE);
        } 
        else {
          ht1632_plot(j+(8*(x-1))-1,i+8,BLACK);
        }
      }
    }
  }
}






