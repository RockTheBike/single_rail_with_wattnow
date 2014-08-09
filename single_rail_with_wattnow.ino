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
 * 2.2 -- now called single_rail_with_wattnow.ino (previously named / forked from Single_Rail_with_WattNow_Fariche.pde)
 * 2.3 --- new branch ppp or pedal powered projector, for 19v buck-converter with addressible LEDs
 */

#define BAUDRATE 57600

const char versionStr[] = "Pedal Power Utility Box ver. 2.3 . single_rail_with_wattnow.ino branch:ppp";

float wattAvg,wattAvgAdder = 0;  // used for averaging wattage for wattnow sign
#define WATTAVGCOUNT 10 // how many main loops to average wattAvgAdder into wattAvg
int wattAvgIndex = 0;  // how many times we have counted wattage

float wattHours = 0;  // how many watt-hours since bootup
unsigned long lastWattHours = 0; // the last time we measured watt-hours

#define numLevels 6
// voltages at which to turn on each level
float levelVolt[numLevels] = {
  16.0, 18.0, 20.0, 22.0, 23.0, 24.0};

#include <Adafruit_NeoPixel.h>

#define LEDPIN 12
#define NUM_LEDS 46
#define BRIGHTNESS 64

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LEDPIN, NEO_GRB + NEO_KHZ800);
// uint32_t ledColor[NUM_LEDS]; // these must be defined in setup() using strip.Color()
// uint32_t offColor[NUM_LEDS] = {0}; // these must be defined in setup() using strip.Color()
uint32_t red,green,blue,white,off; // these must be defined in setup() using strip.Color()

// Arduino pins to be used as inputs to voltage sensor. This shows where to connect the wires! 
#define VOLTPIN A0 // Voltage Sensor Input
#define AMPSPIN A3 // AC power and all DC devices.
#define RELAYPIN 2
#define VOLTCOEFF 13.179 // correct value for new blue arbduino v2
#define AMPCOEFF 8.0682 // 583 - 512 = 71; 71 / 8.8 amps = 8.0682
#define AMPOFFSET 512 // when current sensor is at 0 amps this is the ADC value
#define MAXVOLT 24.3
#define COMEBACKINVOLTAGE 22.0

// vars to store temp values
int adcvalue = 0;

//Voltage related variables. 
float voltage = 0;

//Current related variables
float amps=0;
int ampsRaw;

float Watts;
float wattage;

int desiredState; // all the LEDs are the same color for now

#define AVG_CYCLES 7 // average measured voltage over this many samples
#define DISPLAY_INTERVAL_MS 1000 // when auto-display is on, display every this many milli-seconds

// timing variables for blink. Mark, there are too many of these. 
int blinkState=0;
int fastBlinkState=0;
unsigned long time = 0;
unsigned long lastFastBlinkTime = 0;
unsigned long lastBlinkTime = 0;
unsigned long timeRead = 0;
unsigned long timeDisplay = 0;

void setup() {
  Serial.begin(BAUDRATE);
  Serial.println(versionStr);

  pinMode(RELAYPIN, OUTPUT);
  pinMode(LEDPIN,OUTPUT);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  red   = strip.Color(BRIGHTNESS,0,0);
  green = strip.Color(0,BRIGHTNESS,0);
  blue  = strip.Color(0,0,BRIGHTNESS);
  white = strip.Color(BRIGHTNESS,BRIGHTNESS,BRIGHTNESS);
  off   = strip.Color(0,0,0);

  getVoltages();

}

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

  if(time - timeDisplay > DISPLAY_INTERVAL_MS){
    timeDisplay = time;
    printDisplay(); // print stuff to serial port about everything
  }
  //First deal with the blink  

  if (((time - lastBlinkTime) > 600) && blinkState==1){
    blinkState=0;
    lastBlinkTime=time;
  } 
  else if (((time - lastBlinkTime) > 600) && blinkState==0){
    blinkState=1; 
    lastBlinkTime=time;
  }


  if (((time - lastFastBlinkTime) > 120) && fastBlinkState==1){
    fastBlinkState=0;
    lastFastBlinkTime=time;
  } 
  else if (((time - lastFastBlinkTime) > 120) && fastBlinkState==0){
    fastBlinkState=1; 
    lastFastBlinkTime=time;
  }  

  //protect the ultracapacitors if needed
  if (voltage > MAXVOLT){
    digitalWrite(RELAYPIN, HIGH);
  }

  if (digitalRead(RELAYPIN) && voltage < COMEBACKINVOLTAGE){
    digitalWrite(RELAYPIN, LOW);
  } 

  //Set the desired lighting states. 
  desiredState = 0; // default to state 0
  for(int i = 0; i < numLevels; i++) {  // what to do if voltage points are met
    if (voltage > levelVolt[i]) desiredState = i+1;
  }

  switch (desiredState) {
  case 0: // voltage below 16.0
    setLeds(off);
    break;
  case 1: // voltage above 16.0
    setLeds(blinkState*green);
    break;
  case 2: // voltage above 18.0
    setLeds(green);
    break;
  case 3: // voltage above 20.0
    setLeds(blue);
    break;
  case 4: // voltage above 22.0
    setLeds(white);
    break;
  case 5: // voltage above 23.0
    setLeds(red);
    break;
  case 6: // voltage above 24.0
    setLeds(fastBlinkState*red);
    break;
  delay(10);
  }
}

void setLeds(uint32_t color) {
  for (int c = 0; c < NUM_LEDS; c++) {
    strip.setPixelColor(c,color);
  }
  strip.show();
}

void getVoltages(){

  voltage = average((analogRead(VOLTPIN) / VOLTCOEFF), voltage); //Felt slow so I am putting in the line below with no averaging

  amps = (float) ((analogRead(AMPSPIN) - AMPOFFSET) / AMPCOEFF);
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
  Serial.print(analogRead(VOLTPIN));
  if (digitalRead(RELAYPIN)) Serial.print(" PROTECTED ");
  //  Serial.print("), amps: ");
  //  Serial.print(amps);
  //  Serial.print(" (");
  //  Serial.print(analogRead(AMPSPIN));
  //  Serial.print("), WattHours: ");
  //  Serial.print(wattHours);
  //  Serial.print(", WattAvg: ");
  //  Serial.print(wattAvg);
  //  Serial.print(", Wattage: ");
  //  Serial.print(voltage * amps);
  Serial.println();
}
