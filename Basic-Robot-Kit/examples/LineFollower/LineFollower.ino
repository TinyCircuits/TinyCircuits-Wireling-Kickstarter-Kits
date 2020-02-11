/**********************************************************************
 * Tiny Car Robot Line Follower
 * This program uses 2 Color Sensors, a Time-of-Flight Distance
 * Sensor, a 0.96" Screen, and 2 stepper motors with the RobotZero 
 * processor to create a car robot that follows a line with the ability
 * to stop quickly when obstacles are in the path.
 * 
 * Hardware by: TinyCircuits
 * Written by: Ben Rose for TinyCircuits
 * 
 * Initialized: Dec 2019
 * Last modified: Jan 2020
 **********************************************************************/
#include <Wire.h>
#include <Wireling.h>
#include <MotorDriver.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"  // The library used for the Color Sensor
#include "VL53L0X.h"    // For interfacing with the Time-of-Flight Distance sensor

#include <TinierScreen.h>
#include <GraphicsBuffer.h>

TinierScreen display = TinierScreen(TinierScreen096);
GraphicsBuffer screenBuffer = GraphicsBuffer(128, 64, colorDepth1BPP);

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs0 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_60X);
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_60X);

MotorDriver servo(15); //value passed is the address- remove resistor R1 for 1, R2 for 2, R1 and R2 for 3

VL53L0X distanceSensor; // Name of sensor
const int tofPort = 3;  // Port # of sensor (Found on Wireling Adapter Board)

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif


// Port assignments
int leftSensorPort = 1;
int rightSensorPort = 2;
int displayPort = 0;
int resetPin = A0 + displayPort;


void setup() {
  delay(100);

  SerialMonitorInterface.begin(115200);

  //while (!SerialMonitorInterface);

  Wire.begin();
  delay(100);
  Wireling.begin();

  stepperInit();
  delay(100);

  //Reset servo driver
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  delay(5);
  digitalWrite(9, LOW);
  delay(5);
  digitalWrite(9, HIGH);
  delay(100);
  // Set the period to 20000us or 20ms, correct for driving most servos
  if (servo.begin(20000)) {
    SerialMonitorInterface.println("Motor driver not detected!");
  }

  Wireling.selectPort(leftSensorPort);
  if (tcs0.begin()) {
    SerialMonitorInterface.println("Found sensor 0");
    // Enable the color sensor LED
    tcs0.setInterrupt(true);
  } else {
    SerialMonitorInterface.println("No TCS34725 found ... check your connections");
    //while (1);
  }

  delay(100);

  Wireling.selectPort(rightSensorPort);
  if (tcs1.begin()) {
    SerialMonitorInterface.println("Found sensor 1");
    // Enable the color sensor LED
    tcs1.setInterrupt(true);
  } else {
    SerialMonitorInterface.println("No TCS34725 found ... check your connections");
    //while (1);
  }


  Wireling.selectPort(displayPort);
  display.begin(resetPin);
  display.setFlip(true);
  if (screenBuffer.begin()) {
    //memory allocation error- buffer too big!
  }
  screenBuffer.setFont(thinPixel7_10ptFontInfo);

  Wireling.selectPort(tofPort);

  // Initialize the distance sensor and set a timeout
  distanceSensor.init();
  distanceSensor.setTimeout(5);
  distanceSensor.setMeasurementTimingBudget(200000);
  distanceSensor.startContinuous();

  // Set the current for the stepper motors
  setMotorCurrent(80);
}


int lastSetSpeedLeft = 0;
int lastSetSpeedRight = 0;

unsigned long motorUpdateInterval = 100;
unsigned long lastMotorUpdate = 0;

// Variables to hold the values the sensor reads
uint16_t r0, g0, b0, c0, luxLeft;
uint16_t r1, g1, b1, c1, luxRight;
int luxMin = 500;
int luxMax = 3000;

const int amtLeftSensorSamples = 32;
int leftSensorSampleBuff[amtLeftSensorSamples];
int leftSensorSampleBuffPos = 0;

const int amtRightSensorSamples = 32;
int rightSensorSampleBuff[amtRightSensorSamples];
int rightSensorSampleBuffPos = 0;

const int amtTOFsensorSamples = 32;
int TOFsensorSampleBuff[amtTOFsensorSamples];
int TOFsensorSampleBuffPos = 0;

int TOFlastGoodMeasurement = 0;

void loop() {
  if (millis() - lastMotorUpdate > motorUpdateInterval) {
    lastMotorUpdate = millis();

    Wireling.selectPort(leftSensorPort);
    tcs0.getRawData(&r0, &g0, &b0, &c0);
    luxLeft = constrain(tcs0.calculateLux(r0, g0, b0), luxMin, luxMax);

    Wireling.selectPort(rightSensorPort);
    tcs1.getRawData(&r1, &g1, &b1, &c1);
    luxRight = constrain(tcs1.calculateLux(r1, g1, b1), luxMin, luxMax);

    luxLeft = map(luxLeft, luxMin, luxMax, 0, 100);
    luxRight = map(luxRight, luxMin, luxMax, 0, 100);
    float Motor1Speed = 8, Motor2Speed = 8;
    float steeringBias = (float)(100.0 - luxLeft) - (float)(100.0 - luxRight);
    if (steeringBias > 0) {
      Motor1Speed -= steeringBias / 7.0;
      // if(abs(steeringBias)>50){
      Motor2Speed += steeringBias / 50.0;
      //}
    } else {
      // if(abs(steeringBias)>50){
      Motor1Speed -= steeringBias / 50.0;
      //}
      Motor2Speed += steeringBias / 7.0;
    }

    Wireling.selectPort(tofPort);
    unsigned int TOFdistance = distanceSensor.readRangeContinuousMillimeters();
    if (TOFdistance != 65535) {
      TOFlastGoodMeasurement = TOFdistance;
    }

    if (TOFlastGoodMeasurement < 50) {
      Motor2Speed = 0;
      Motor1Speed = 0;
    }

    if (Motor2Speed != lastSetSpeedLeft) {
      setMotor(1, Motor2Speed);
      lastSetSpeedLeft = Motor2Speed;
    }
    if (Motor1Speed != lastSetSpeedRight) {
      setMotor(2, Motor1Speed);
      lastSetSpeedRight = Motor1Speed;
    }

    /*
        SerialMonitorInterface.print(Motor2Speed);
        SerialMonitorInterface.print('\t');
        SerialMonitorInterface.print(Motor1Speed);
        SerialMonitorInterface.print('\t');
        SerialMonitorInterface.print(luxLeft);
        SerialMonitorInterface.print('\t');
        SerialMonitorInterface.print(luxRight);
        SerialMonitorInterface.print('\t');
        SerialMonitorInterface.print(steeringBias);
        SerialMonitorInterface.print('\t');
        SerialMonitorInterface.println(TOFlastGoodMeasurement);
    */
    
    Wireling.selectPort(displayPort);
    screenBuffer.clear();
    screenBuffer.setCursor(0, 43);
    screenBuffer.print("M2: ");
    screenBuffer.print(Motor2Speed);
    screenBuffer.setCursor(90, 43);
    screenBuffer.print("M1: ");
    screenBuffer.print(Motor1Speed);
    screenBuffer.setCursor(0, 53);
    screenBuffer.print("Steering bias: ");
    screenBuffer.print(steeringBias);

    screenBuffer.setCursor(0, 0);
    screenBuffer.print("Lux:");
    screenBuffer.setCursor(127 - 32, 0);
    screenBuffer.print("Lux:");
    screenBuffer.setCursor(64 - 16, 0);
    screenBuffer.print("Dist:");

    if ((int)Motor2Speed == 0 && (int)Motor1Speed == 0) {
      screenBuffer.setCursor(44, 43);
      screenBuffer.print("Stopped!");
    }
    
    leftSensorSampleBuff[leftSensorSampleBuffPos] = luxLeft;
    rightSensorSampleBuff[rightSensorSampleBuffPos] = luxRight;
    TOFsensorSampleBuff[TOFsensorSampleBuffPos] = constrain(TOFlastGoodMeasurement, 0, 1000);

    leftSensorSampleBuffPos++;
    if (leftSensorSampleBuffPos >= amtLeftSensorSamples) leftSensorSampleBuffPos = 0;
    rightSensorSampleBuffPos++;
    if (rightSensorSampleBuffPos >= amtRightSensorSamples) rightSensorSampleBuffPos = 0;
    TOFsensorSampleBuffPos++;
    if (TOFsensorSampleBuffPos >= amtTOFsensorSamples) TOFsensorSampleBuffPos = 0;
    int graphY = 10;
    displayGraph(0, graphY, 32, 32, 0, 100, rightSensorSampleBuff, rightSensorSampleBuffPos);
    displayGraph(127 - 32, graphY, 32, 32, 0, 100, leftSensorSampleBuff, leftSensorSampleBuffPos);
    displayGraph(64 - 16, graphY, 32, 32, 0, 1000, TOFsensorSampleBuff, TOFsensorSampleBuffPos);

    Wire.setClock(1000000);
    display.writeBuffer(screenBuffer.getBuffer(), screenBuffer.getBufferSize());
    Wire.setClock(100000);
  }
}


void displayGraph(int xDispPos, int yDispPos, int width, int height, int dataMin, int dataMax, int * buffToDisplay, int buffToDisplayPos) {

  screenBuffer.drawLine(xDispPos, yDispPos, xDispPos + width, yDispPos, 0xFFFF);
  screenBuffer.drawLine(xDispPos, yDispPos, xDispPos, yDispPos + height, 0xFFFF);
  screenBuffer.drawLine(xDispPos + width, yDispPos, xDispPos + width, yDispPos + height, 0xFFFF);
  screenBuffer.drawLine(xDispPos, yDispPos + height, xDispPos + height, yDispPos + height, 0xFFFF);

  for (uint8_t i = 1; i < width; i++) {
    int sample = map(buffToDisplay[(buffToDisplayPos + (i)) % width], dataMin, dataMax, 0, height);
    int sample0 = map(buffToDisplay[(buffToDisplayPos + (i - 1)) % width], dataMin, dataMax, 0, height);
    screenBuffer.drawLine(xDispPos + i - 1, yDispPos + height - sample0, xDispPos + i, yDispPos + height - sample, 0xFFFF);
  }
}
