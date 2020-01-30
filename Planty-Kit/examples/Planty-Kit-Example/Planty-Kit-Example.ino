/************************************************************************
 * Wireling Starter Kit Shipment Program
 * This program uses four of the Wirelings included with the Starter Kit:
 * Port 0: 0.69" OLED Screen Wireling
 * Port 1: Buzzer Wireling
 * Port 2: Light Sensor Wireling
 * Port 3: Soil Moisture Sensor Wireling
 * 
 * When plugged in according to the above mapping, the 0.69" Screen will 
 * display the lux values read from the light sensor, and will also display 
 * the temperature and moisture detected by the soil moisture sensor Wireling.
 * While the moisture value is less than 10%, the buzzer will sound every 30
 * seconds. The buzzer will also sound once at startup.
 *
 * Hardware by: TinyCircuits
 * Written by: Hunter Hykes for TinyCircuits
 *
 * Initiated: 12/26/2019 
 * Updated: 12/27/2019
 ************************************************************************/

#include <Wire.h>               // For I2C communication with sensor
#include <Wireling.h>
#include <TinierScreen.h>       // For interfacing with the 0.69" OLED
#include <GraphicsBuffer.h>
#include "pitches.h"

// Make compatible with all TinyCircuits processors
#if defined(ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

/* * * * * * * * * * 0.69" OLED * * * * * * * * * */
#define OLED_PORT 0 // use Port 0 for screen
#define OLED_RST A0 //OLED reset line
#define OLED_069_WIDTH 96
#define OLED_069_HEIGHT 16
TinierScreen display069 = TinierScreen(TinierScreen069);
GraphicsBuffer screenBuffer069 = GraphicsBuffer(OLED_069_WIDTH, OLED_069_HEIGHT, colorDepth1BPP);

/* * * * * * * * * * BUZZER * * * * * * * * * */
#define pin (uint8_t) A1

/* * * * * * * * * * * LIGHT SENSOR * * * * * * * * * * * */
#define LIGHT_PORT 2
#define TSL2572_I2CADDR     0x39
#define   GAIN_1X 0
#define   GAIN_8X 1
#define  GAIN_16X 2
#define GAIN_120X 3

//only use this with 1x and 8x gain settings
#define GAIN_DIVIDE_6 true

// Global variable for gain value used to Read the sensor
int gain_val = 0;

/* * * * * * * * * * MOISTURE * * * * * * * * * * */
#define MOISTURE_PORT 3
#define MINCAPREAD 710
#define MAXCAPREAD 975
#define ANALOGREADMAX 1023
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define BCOEFFICIENT 3380
#define SERIESRESISTOR 35000

// Simple templated averaging class based on Running Average by Rob Tillaart: http://arduino.cc/playground/Main/RunningAverage
template <const unsigned int N>
class RunningAverageFloat
{
  public:
    void addValue(float val) {
      _ar[_index] = val;
      _index++;
      if (_index == N) _index = 0;
    };
    void fillValue(float val) {
      for (unsigned int i = 0; i < N; i++)_ar[i] = val;
    };
    float getAverage() {
      float sum = 0.0;
      for (unsigned int i = 0; i < N; i++)sum += _ar[i];
      return sum / (float)N;
    };
  protected:
    int _index = 0;
    float _ar[N];
};

RunningAverageFloat<35> moistureAverage;
RunningAverageFloat<35> temperatureAverage;
RunningAverageFloat<35> luxAverage;

int melody[] = { // Notes in the melody:
  NOTE_C4, NOTE_C3, NOTE_C4, NOTE_C3
};

int noteDurations[] = { // Note durations: 4 = quarter note, 8 = eighth note, etc.:
  16, 16, 16, 16
};

void setup(void) {
  SerialMonitorInterface.begin(115200);
  Wire.begin();
  Wireling.begin(); // Enable power & select port
  delay(200); // boot sensor
  
  /* * * * * * Screen Stuff * * * * */
  Wireling.selectPort(OLED_PORT);
  display069.begin(OLED_RST);
  if (screenBuffer069.begin()) {
    //memory allocation error- buffer too big!
  }
  screenBuffer069.setFont(thinPixel7_10ptFontInfo);
  
  /* * * * * Light Sensor Stuff * * * * */
  Wireling.selectPort(LIGHT_PORT);
  TSL2572Init(GAIN_16X);
  luxAverage.fillValue(Tsl2572ReadAmbientLight());

  /* * * * * Buzzer * * * * */
  beepBoop(); //run buzzer at startup for funsies

  /* * * * * Moisture Sensor * * * * */
  Wireling.selectPort(MOISTURE_PORT);
  moistureAverage.fillValue(readMoisture());
  temperatureAverage.fillValue(readTemp());
}

int x, y, moistureLevel;
float luxLevel, tempLevel, oldTime, deltaTime(30000), startTime, updateOLED(250);
String moisture, lux, temp;

void loop(void) {
  startTime = millis();
  
  do{
  // buzzer goes off every 30 seconds if moistureLevel < 10
  if(moistureLevel < 10) {
    if(millis() - oldTime > deltaTime) {
      beepBoop();
      oldTime = millis();
    }
  }
  
  Wireling.selectPort(MOISTURE_PORT);
  moistureAverage.addValue(readMoisture()); // get moisture
  moistureLevel = moistureAverage.getAverage();
  moisture = String(moistureLevel); // update the string for the screen display
  temperatureAverage.addValue(readTemp());
  tempLevel = temperatureAverage.getAverage();
  temp = String(tempLevel);
  delay(5);

  Wireling.selectPort(LIGHT_PORT);
  luxAverage.addValue(Tsl2572ReadAmbientLight());
  luxLevel = luxAverage.getAverage();
  lux = String(luxLevel);
  delay(5);
  
  }while(millis() - startTime < updateOLED);

  Wireling.selectPort(OLED_PORT);           // select the Wireling screen port
  screenBuffer069.clear();                  // clear old screen contents
  printMoisture();
  printLuxToOLED();
  display069.writeBuffer(screenBuffer069.getBuffer(), screenBuffer069.getBufferSize()); // write buffer to the screen
}

// print light sensor info to TinierScreen
void printLuxToOLED() {
  screenBuffer069.setCursor(x = 56, y = 7); // set cursor to (0, 0)
  screenBuffer069.print("L: " + lux);     // print Lux Value
}

void printMoisture() {
  screenBuffer069.setCursor(x = 0, y = -1);
  screenBuffer069.print("M: " + moisture + "%");
  if(moistureLevel < 10) {
    screenBuffer069.print(" Water Me!");
  }
  screenBuffer069.setCursor(x = 0, y = 7);
  screenBuffer069.print("T: " + temp + "C");
}

int readMoisture(){
  Wire.beginTransmission(0x30);
  Wire.write(1);
  Wire.endTransmission();
  delay(5);
  int c=0;
  Wire.requestFrom(0x30, 2);
  if(Wire.available()==2)
  { 
    c = Wire.read();
    c <<= 8;
    c |= Wire.read();
    c = constrain(c, MINCAPREAD, MAXCAPREAD);
    c = map(c, MINCAPREAD, MAXCAPREAD, 0, 100);
  }
  return c;
}

float readTemp() {
  Wire.beginTransmission(0x30);
  Wire.write(2);
  Wire.endTransmission();
  delay(5);
  int c = 0;
  Wire.requestFrom(0x30, 2);
  if (Wire.available() == 2)
  {
    c = Wire.read();
    c <<= 8;
    c |= Wire.read();
    //https://learn.adafruit.com/thermistor/using-a-thermistor thanks!
    uint32_t adcVal = ANALOGREADMAX - c;
    uint32_t resistance = (SERIESRESISTOR * ANALOGREADMAX) / adcVal - SERIESRESISTOR;
    float steinhart = (float)resistance / THERMISTORNOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C
    return steinhart;
  }
  return c;
}

void beepBoop() {
  // Iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 4; thisNote++) {

    // To calculate the note duration, take one second divided by the note type.
    // Ex. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote]; // should be 1000, but 1250 sounds nicer for this song
    tone(pin, melody[thisNote], noteDuration);

    // To distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // Stop the tone playing:
    noTone(8);
  }
}

// Used to interface with the sensor by writing to its registers directly 
void Tsl2572RegisterWrite(byte regAddr, byte regData) {
  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0x80 | regAddr);
  Wire.write(regData);
  Wire.endTransmission();
}

// Initializes the light sensor to be ready for output
void TSL2572Init(uint8_t gain) {
  Tsl2572RegisterWrite( 0x0F, gain );//set gain
  Tsl2572RegisterWrite( 0x01, 0xED );//51.87 ms
  Tsl2572RegisterWrite( 0x00, 0x03 );//turn on
  if (GAIN_DIVIDE_6)
    Tsl2572RegisterWrite( 0x0D, 0x04 );//scale gain by 0.16
  if (gain == GAIN_1X)gain_val = 1;
  else if (gain == GAIN_8X)gain_val = 8;
  else if (gain == GAIN_16X)gain_val = 16;
  else if (gain == GAIN_120X)gain_val = 120;
}

// Read the lux value from the light sensor so we can print it out
float Tsl2572ReadAmbientLight() {
  Wireling.selectPort(LIGHT_PORT);
  
  uint8_t data[4];
  int c0, c1;
  float lux1, lux2, cpl;

  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0xA0 | 0x14);
  Wire.endTransmission();
  Wire.requestFrom(TSL2572_I2CADDR, 4);
  for (uint8_t i = 0; i < 4; i++)
    data[i] = Wire.read();

  c0 = data[1] << 8 | data[0];
  c1 = data[3] << 8 | data[2];

  //see TSL2572 datasheet: https://www.mouser.com/ds/2/588/TSL2672_Datasheet_EN_v1-255694.pdf
  cpl = 51.87 * (float)gain_val / 60.0;
  if (GAIN_DIVIDE_6) cpl /= 6.0;
  lux1 = ((float)c0 - (1.87 * (float)c1)) / cpl;
  lux2 = ((0.63 * (float)c0) - (float)c1) / cpl;
  cpl = max(lux1, lux2);
  return max(cpl, 0.0);
}
