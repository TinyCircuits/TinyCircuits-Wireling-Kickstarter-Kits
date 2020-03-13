//-------------------------------------------------------------------------------
//  TinyCircuits Robot Kit Examples
//  Last Updated 24 Feb 2020
//
//  This example includes line following and object detecting code for the
//  RobotZero processor board in the robot car and arm kits. The sensors detected
//  at startup determine which example runs. The line following code attemps to
//  follow a 5 to 10mm wide dark line drawn on a light sheet. The object detecting
//  code attempts to find a small object like a pen cap to pick up and throw.
//
//  Written by Ben Rose for TinyCircuits, https://tinycircuits.com
//
//-------------------------------------------------------------------------------


#include <Wire.h>
#include <Wireling.h>
#include <ATtiny841Lib.h>
#include <ServoDriver.h>
#include <MotorDriver.h>
#include <SPI.h>
#include <TinierScreen.h>
#include <GraphicsBuffer.h>
#include "Adafruit_TCS34725.h"
#include "VL53L0X.h"

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif


TinierScreen display = TinierScreen(TinierScreen096);
GraphicsBuffer screenBuffer = GraphicsBuffer(128, 64, colorDepth1BPP);

ServoDriver servo(ROBOTZERO_SERVO_ADDR); //value passed is the address- remove resistor R1 for 1, R2 for 2, R1 and R2 for 3

VL53L0X distanceSensor; // Name of sensor
int tofPort = 0;  // Port # of sensor (Found on Wireling Adapter Board)

int displayPort = 1;
int resetPin = A0 + displayPort;


unsigned int getTOFMeasurement() {
  Wireling.selectPort(tofPort);
  unsigned int TOFdistance = distanceSensor.readRangeContinuousMillimeters();
  while (TOFdistance == 65535) {
    TOFdistance = distanceSensor.readRangeContinuousMillimeters();
  }
  return TOFdistance;
}




/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs0 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_60X);
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_60X);

int leftSensorPort = 1;
int rightSensorPort = 2;


//1000=open
const int clawServo = 1;
//1000 = right
const int rotationServo = 2;
//1000 = back
const int frontBackServo = 3;
//1000 = down
const int upDownServo = 4;

int clawClosePosition = 2200;
int clawOpenPosition = 1000;

int rotationServoN45Pos = 1000;
int rotationServoP45Pos = 2050;

int scanningHeightServoPosition = 1000;


void setup() {
  SerialMonitorInterface.begin(115200);
  Wire.begin();
  delay(10);
  Wireling.begin();

  stepperInit();
  delay(100);

  servo.useResetPin();
  if (servo.begin(10000)) {
    while (1) {
      SerialMonitorInterface.println("Servo driver not detected!");
      delay(1000);
    }
  }
  servo.setServo(1, 1500);
  servo.setServo(2, 1500);
  servo.setServo(3, 1500);
  servo.setServo(4, 1500);

  delay(1000);
  setMotorCurrent(80);

  //try to init color sensors, change screen port to 0 for line follower example
  bool noColorSensor = false;
  Wireling.selectPort(leftSensorPort);
  if (tcs0.begin()) {
    tcs0.setInterrupt(true);
    Wireling.selectPort(rightSensorPort);
    if (tcs1.begin()) {
      tcs1.setInterrupt(true);
      displayPort = 0;
      resetPin = A0 + displayPort;
      tofPort = 3;
    } else {
      while (1) {
        SerialMonitorInterface.println("Unexpected Wireling configuration! 1");
        delay(1000);
      }
    }
  } else {
    noColorSensor = true;
  }

  Wireling.selectPort(displayPort);
  if (noColorSensor) {
    Wire.beginTransmission(SSD1306_DEFAULT_ADDRESS);
    if (Wire.endTransmission()) {
      while (1) {
        SerialMonitorInterface.println("Unexpected Wireling configuration! 2");
        delay(1000);
      }
    }
  }
  display.begin(resetPin);
  display.setFlip(true);
  if (screenBuffer.begin()) {
    //memory allocation error- buffer too big!
  }
  screenBuffer.setFont(thinPixel7_10ptFontInfo);


  Wireling.selectPort(tofPort);
  // Initialize the distance sensor and set a timeout
  if (distanceSensor.init()) {
    distanceSensor.setTimeout(5);
    distanceSensor.setMeasurementTimingBudget(100000);
    distanceSensor.startContinuous();
  }

  closeClaw(200);
  servo.setServo(frontBackServo, 2200);
  servo.setServo(upDownServo, scanningHeightServoPosition);
}



void loop() {
  if (displayPort == 0) {
    lineFollowerLoop();
  } else {
    robotArmLoop();
  }
}


int minAngle = 0;
int minDistance = 0;

void scanForObject() {
  minDistance = 1000;
  int degreesScanned = 0;
  int degreeStep = 1;
  bool isDecreasing = false;
  int decreasingCount = 0;
  bool isIncreasing = false;
  int increasingCount = 0;
  while (1) {
    while (isMotorSpinning());
    turnRobotDegree(degreeStep, 4, false);
    Wireling.selectPort(tofPort);
    unsigned int TOFdistance = getTOFMeasurement();

    int val = constrain(TOFdistance, 0, 1000);
    val = TOFdistance * 64 / 300;
    screenBuffer.clear();
    screenBuffer.setCursor(0, 0);
    screenBuffer.print("Scanning for object! ");
    screenBuffer.setCursor(0, 10);
    screenBuffer.print("Distance reading: ");
    screenBuffer.print(TOFdistance);

    Wireling.selectPort(displayPort);
    Wire.setClock(1000000);
    display.writeBuffer(screenBuffer.getBuffer(), screenBuffer.getBufferSize());
    Wire.setClock(100000);

    if (TOFdistance > 80 && TOFdistance < 300) {
      if (TOFdistance < minDistance) {
        if (minAngle == degreesScanned - degreeStep) {
          decreasingCount++;
        } else {
          decreasingCount = 1;
        }
        minDistance = TOFdistance;
        minAngle = degreesScanned;
      } else {
        if (decreasingCount > 3) {
          return;
        } else {
          decreasingCount = 0;
          minDistance = 1000;
        }
      }
    } else {
      decreasingCount = 0;
      minDistance = 1000;
    }
    degreesScanned += degreeStep;
  }
}

int leftRightScan(int degrees) {
  turnRobotDegree(-degrees, 7, true);

  minDistance = 1000;
  screenBuffer.clear();


  for (int i = -degrees; i < degrees; i += 1) {
    while (isMotorSpinning());
    turnRobotDegree(1, 4, false);
    Wireling.selectPort(tofPort);
    unsigned int TOFdistance = getTOFMeasurement();
    int val = constrain(TOFdistance, 0, 1000);
    val = TOFdistance * 64 / 300;
    screenBuffer.drawLine(64 + (degrees * 2) - (degrees + i), 63, 64 + (degrees * 2) - (degrees + i), val, 0xFFFF);
    screenBuffer.drawLine(64 + (degrees * 2) - (degrees + i), val + 1, 64 + (degrees * 2) - (degrees + i), 0, 0);
    screenBuffer.setCursor(0, 0);
    screenBuffer.print("Scanning.. ");
    Wireling.selectPort(displayPort);
    Wire.setClock(1000000);
    display.writeBuffer(screenBuffer.getBuffer(), screenBuffer.getBufferSize());
    Wire.setClock(100000);
    if (TOFdistance < minDistance) {
      minDistance = TOFdistance;
      minAngle = i;
    }
  }
}

void robotArmLoop() {
  scanForObject();

  if (minDistance > 60 && minDistance < 300) {
    screenBuffer.clear();
    screenBuffer.setCursor(0, 0);
    screenBuffer.print("Object found ");
    screenBuffer.print(minDistance);
    screenBuffer.print(" mm away!");

    Wireling.selectPort(displayPort);
    Wire.setClock(1000000);
    display.writeBuffer(screenBuffer.getBuffer(), screenBuffer.getBufferSize());
    Wire.setClock(100000);

    turnRobotDegree(-3, 5, true);

    moveRobotForwardMM(max(0, (float)minDistance * 0.75 - 55.0));
    delay(100);
    servo.setServo(upDownServo, scanningHeightServoPosition - 100);
    leftRightScan(10);
    if (minDistance > 50 && minDistance < 100) {
      turnRobotDegree(-10 + minAngle + 2, 7, true);
      if (minDistance > 55)
        moveRobotForwardMM(max(0, minDistance - 55));
      delay(100);
      unsigned int TOFdistance = getTOFMeasurement();
      if (TOFdistance > 45 && TOFdistance < 100) {
        int distanceMoved = 0;
        while (TOFdistance > 45 && distanceMoved < 50) {
          while (isMotorSpinning());
          moveRobotForward(1, 5, false);
          TOFdistance = getTOFMeasurement();
          distanceMoved++;
        }
        closeClaw(1000);
        delay(100);
        servo.setServo(upDownServo, scanningHeightServoPosition);
        delay(500);
        rotateArmSlowly(0, -45);
        delay(1000);

        rotateArm(45);
        delay(50);
        servo.setServo(upDownServo, 1600);
        delay(100);
        servo.setServo(clawServo, 1000);
        delay(1000);
        rotateArmSlowly(45, 0);
        servo.setServo(upDownServo, scanningHeightServoPosition);
      }

    } else {
      //object too close/far to pick up
    }
  }
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

void lineFollowerLoop() {
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
