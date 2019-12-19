/************************************************************************
 * Wireling Starter Kit Shipment Program
 * This program uses four of the Wirelings included with the Starter Kit:
 * Port 0: 0.42" OLED Screen Wireling
 * Port 1: RGB LED Wireling
 * Port 2: TOF Sensor Wireling
 * Port 3: Color Sensor Wireling
 * 
 * When plugged in according to the above mapping, the 0.42" Screen will 
 * display the RGB values read from the color sensor, and will also display 
 * the distance detected by the TOF sensor Wireling in mm. The RGB LED
 * Wireling will be updated to the RGB values read by the color sensor.
 *
 * Hardware by: TinyCircuits
 * Written by: Hunter Hykes for TinyCircuits
 *
 * Initiated: 12/18/2019 
 * Updated: 12/19/2019
 ************************************************************************/

#include <Wire.h>               // For I2C communication with sensor
#include <Wireling.h>
#include <TinierScreen.h>       // For interfacing with the 0.42" OLED
#include <TinyBuffer.h>         // For building a screen buffer for the 0.42" OLED
#include "font.h"
#include <FastLED.h>            // For interfacing with the RGB LED
#include "VL53L0X.h"            // For interfacing with the Time-of-Flight Distance sensor
#include <Adafruit_TCS34725.h>  // For interfacing with the Color Sensor

// Make compatible with all TinyCircuits processors
#if defined(ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

/* * * * * * * * * * 0.42" OLED * * * * * * * * * */
#define OLED_PORT 0 // use Port 0 for screen
TinierScreen display042 = TinierScreen(OLED042);
TinyBuffer screenBuffer042 = TinyBuffer(72, 40, colorDepth1BPP);

/* * * * * * * * * * * RGB LED * * * * * * * * * * * */
#define NUM_LEDS 1 //this is the number of LEDs in your strip
#define DATA_PIN (uint8_t) A1
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
int brightness = 20; //value from 0-255 to manipulate brightness

/* * * * * * * * * * TOF Sensor * * * * * * * * * * */
#define TOF_PORT 2 // use Port 2 for TOF sensor
VL53L0X distanceSensor;

/* * * * * * * * * * Color Sensor * * * * * * * * * */
#define TCS_PORT 3 // use Port 3 for color sensor
#define TCS_LIGHT 9 // analog pin on Wireling connector for powering white LEDs
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c, colorTemp, lux; // Variables to hold the values the sensor reads

void setup(void) {
  SerialMonitorInterface.begin(115200);
  Wire.begin();
  Wireling.begin(); // Enable power & select port
  delay(200); // boot sensor
  
  /* * * * * * Screen Stuff * * * * */
  Wireling.selectPort(OLED_PORT);
  display042.begin();
  Wire.setClock(1000000);
  screenBuffer042.setFont(thinPixel7_10ptFontInfo);
  
  /* * * * * RGB LED Stuff * * * * */
  FastLED.addLeds<WS2812B, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(brightness);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(TCS_LIGHT, OUTPUT);

  /* * * * * TOF Stuff * * * * */
  Wireling.selectPort(TOF_PORT);
  distanceSensor.init();
  distanceSensor.setTimeout(500);
  distanceSensor.setMeasurementTimingBudget(200000);
  distanceSensor.startContinuous();

  /* * * * * Color Sensor Stuff * * * * */
  Wireling.selectPort(TCS_PORT); //The port is the number on the Adapter board where the sensor is attached
  tcs.begin();
  LEDon(); // turn on color sensor Wireling LEDs
}

int x, y;
String distance;

void loop(void) {
  Wireling.selectPort(TCS_PORT);
  tcs.getRawData(&r, &g, &b, &c);
  //colorTemp = tcs.calculateColorTemperature(r, g, b);
  //lux = tcs.calculateLux(r, g, b);

  updateRGBLED(r, g, b);
  delay(10);
  getTOFdistance();
  
  printRGBtoOLED();
  printTOFdistance();
  display042.writeBuffer(screenBuffer042.getBuffer(), screenBuffer042.getBufferSize()); // write buffer to the screen
}

void updateRGBLED(uint16_t red, uint16_t green, uint16_t blue) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(green, red, blue); // GRB
    FastLED.show(); //update the LEDs
  }
}

// print color sensor info to TinierScreen
void printRGBtoOLED() {
  Wireling.selectPort(OLED_PORT);           // select the Wireling screen port
  screenBuffer042.clear();                  // clear old screen contents
  screenBuffer042.setCursor(x = 24, y = 0);  // set cursor to (0, 0)
  screenBuffer042.print("R: " + String(r)); // print Red Value
  screenBuffer042.setCursor(x, y += 8);     // advance to next line (font height is 7 so use 8)
  screenBuffer042.print("G: " + String(g)); // print Green Value
  screenBuffer042.setCursor(x, y += 8);     // advance to next line
  screenBuffer042.print("B: " + String(b)); // print Blue Value
}

void getTOFdistance() {
  Wireling.selectPort(TOF_PORT);
  distance = String(distanceSensor.readRangeContinuousMillimeters());
}

void printTOFdistance() {
  Wireling.selectPort(OLED_PORT);           // select the Wireling screen port
  //screenBuffer042.clear();
  screenBuffer042.setCursor(x = 10, y = 24); // hard-coded to be last line
  screenBuffer042.print("Dist: " + distance + "mm");
}

// print color sensor info to SerialMonitorInterface
void printRGBReading() {
  SerialMonitorInterface.print("Color Temp: "); SerialMonitorInterface.print(colorTemp); SerialMonitorInterface.print(" K, ");
  SerialMonitorInterface.print("Lux: "); SerialMonitorInterface.print(lux, DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("R: "); SerialMonitorInterface.print(r, DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("G: "); SerialMonitorInterface.print(g, DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("B: "); SerialMonitorInterface.print(b); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("Clr: "); SerialMonitorInterface.print(c, DEC);
  SerialMonitorInterface.println(" ");
}

// Turn Wireling LEDs on
void LEDon() {
  tcs.setInterrupt(true);
}

// Turn Wireling LEDs off
void LEDoff() {
  tcs.setInterrupt(false);
}
