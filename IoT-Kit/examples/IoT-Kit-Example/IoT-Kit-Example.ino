/************************************************************************
 * Wireling Starter Kit Shipment Program
 * This program uses four of the Wirelings included with the Starter Kit:
 * Port 0: 0.96" OLED Screen Wireling
 * Port 1: Soil Moisture Sensor Wireling
 * Port 2: Light Sensor Wireling
 * Port 3: TEMP/PRES/HUM/VOC Wireling
 * 
 * When plugged in according to the above mapping, the 0.96" Screen will 
 * display the lux values read from the light sensor, and will also display 
 * the temperature and moisture detected by the soil moisture sensor Wireling.
 *
 * Hardware by: TinyCircuits
 * Written by: Hunter Hykes for TinyCircuits
 *
 * Initiated: 12/26/2019 
 * Updated: 01/14/2020
 ************************************************************************/

#include <Wire.h>               // For I2C communication with sensor
#include <SPI.h>
#include <Wireling.h>
#include <TinierScreen.h>       // For interfacing with the 0.96" OLED
#include <GraphicsBuffer.h>     // For building a screen buffer for the 0.96" OLED
#include <FastLED.h>            // For interfacing with the RGB LED
#include "Adafruit_Sensor.h"
#include "Adafruit_BME680.h"

// Make compatible with all TinyCircuits processors
#if defined(ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

/* * * * * * * * * * 0.96" OLED * * * * * * * * * */
#define OLED_PORT 0 // use Port 0 for screen
#define OLED_RST (uint8_t) A0 //OLED reset line
TinierScreen display096 = TinierScreen(TinierScreen096);
GraphicsBuffer screenBuffer096 = GraphicsBuffer(128, 64, colorDepth1BPP);

/* * * * * * * * * * MOISTURE * * * * * * * * * * */
#define MOISTURE_PORT 1
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

RunningAverageFloat<5> moistureAverage;
RunningAverageFloat<5> temperatureAverage;

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

/* * * * * * * TEMP/PRES/HUM/VOC * * * * * * * */
#define WEATHER_PORT 3
bool BME680Flag = true;

// Global Sensor Variables
#define SEALEVELPRESSURE_HPA (1013.25) // used to find approximate altitude 
Adafruit_BME680 bme; // I2C

// raw values have suffix "Level" whereas strings do not
int x, y, moistureLevel, counter(0);
float luxLevel, tempLevel, tempLevel2, presLevel, altLevel, humLevel, vocLevel;
String moisture, lux, temp, mTemp, pres, alt, hum, voc;

void setup(void) {
  SerialMonitorInterface.begin(115200);
  Wire.begin();
  Wireling.begin(); // Enable power & select port
  delay(250); // let things power on
  
  /* * * * * * Screen Stuff * * * * */
  Wireling.selectPort(OLED_PORT);
  display096.begin(OLED_RST);
  if (screenBuffer096.begin()) {
    //memory allocation error- buffer too big!
  }
  screenBuffer096.setFont(thinPixel7_10ptFontInfo);
  
  /* * * * * Moisture Sensor * * * * */
  Wireling.selectPort(MOISTURE_PORT);
  moistureAverage.fillValue(readMoisture());
  temperatureAverage.fillValue(readTemp());
  
  /* * * * * Light Sensor Stuff * * * * */
  Wireling.selectPort(LIGHT_PORT);
  TSL2572Init(GAIN_16X);

  /* * * * * TEMP/PRES/HUM/VOC * * * * */
  while (BME680Flag) {
    Wireling.selectPort(WEATHER_PORT);  // select BME680 port again
    BME680Flag = !bme.begin(0x76);      // returns 0 (false) if successful
    
    Wireling.selectPort(OLED_PORT);     // select the Wireling screen port
    screenBuffer096.clear();
    screenBuffer096.setCursor(x = 22, y = 24);
    screenBuffer096.print("No BME680 found.");    // indicate that BME680 was not found
    screenBuffer096.setCursor(x = 55, y = 32);
    screenBuffer096.print(String(counter) + "s"); // write upcounter to screen
    display096.writeBuffer(screenBuffer096.getBuffer(), screenBuffer096.getBufferSize()); // write buffer to the screen

    delay(1000);  //wait one second
    counter++;    // increment counter to show life
  }
  
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop(void) {
  getWeather();   // get environment data from BME680
  getLux();       // get illuminance from ambient light sensor
  getMoisture();  // get soil moisture and soil temperature
  
  Wireling.selectPort(OLED_PORT); // select the Wireling screen port
  screenBuffer096.clear();  // clear old buffer contents
  printWeather();           // write environment data to buffer
  printLux();               // write illuminance data to buffer
  printMoisture();          // write soil data to buffer
  Wire.setClock(1000000);
  display096.writeBuffer(screenBuffer096.getBuffer(), screenBuffer096.getBufferSize()); // write buffer to the screen
  Wire.setClock(50000);
}

void getWeather() {
  Wireling.selectPort(WEATHER_PORT);

  altLevel = bme.readAltitude(SEALEVELPRESSURE_HPA); // This will run performReading(), updating all readings
  alt = String(altLevel);
  
  tempLevel = bme.temperature;
  temp = String(tempLevel);
  
  presLevel = (bme.pressure / 100.0F);
  pres = String(presLevel);
  
  humLevel = bme.humidity;
  hum = String(humLevel);
  
  vocLevel = (bme.gas_resistance / 1000.0);
  voc = String(vocLevel);
}

void printWeather() {
  screenBuffer096.setCursor(x = 0, y = -1);
  screenBuffer096.print("Temperature:");
    screenBuffer096.setCursor(x = 69, y = -1);
    screenBuffer096.print(temp + " *C");
  screenBuffer096.setCursor(x = 0, y = 7);
  screenBuffer096.print("Pressure:");
    screenBuffer096.setCursor(x = 69, y = 7);
    screenBuffer096.print(pres + "hPa");
  screenBuffer096.setCursor(x = 0, y = 15);
  screenBuffer096.print("Alitiude:");
    screenBuffer096.setCursor(x = 69, y = 15);
    screenBuffer096.print(alt + " m");
  screenBuffer096.setCursor(x = 0, y = 23);
  screenBuffer096.print("Humidity:");
    screenBuffer096.setCursor(x = 69, y = 23);
    screenBuffer096.print(hum + " %");
  screenBuffer096.setCursor(x = 0, y = 31);
  screenBuffer096.print("VOC:          ");
    screenBuffer096.setCursor(x = 69, y = 31);
    screenBuffer096.print(voc + " KOhms");
}

void getLux() {
  Wireling.selectPort(LIGHT_PORT);
  luxLevel = Tsl2572ReadAmbientLight();
  lux = String(luxLevel);
}

// print light sensor info to TinierScreen
void printLux() {
  screenBuffer096.setCursor(x = 0, y = 39);  // set cursor to (0, 0)
  screenBuffer096.print("Illuminance:");       // print Lux Value
    screenBuffer096.setCursor(x = 69, y = 39);
    screenBuffer096.print(lux + " lux");
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

void getMoisture() {
  Wireling.selectPort(MOISTURE_PORT);

  moistureAverage.addValue(readMoisture()); // get moisture
  moistureLevel = moistureAverage.getAverage();
  moisture = String(moistureLevel); // update the string for the screen display
  
  temperatureAverage.addValue(readTemp());
  tempLevel2 = temperatureAverage.getAverage();
  mTemp = String(tempLevel2);
}

void printMoisture() {
  screenBuffer096.setCursor(x = 0, y = 47);
  screenBuffer096.print("Soil Moisture:");
    screenBuffer096.setCursor(x = 69, y = 47);
    screenBuffer096.print(moisture + "%");
  screenBuffer096.setCursor(x = 0, y = 55);
  screenBuffer096.print("Soil Temp:");
    screenBuffer096.setCursor(x = 69, y = 55);
    screenBuffer096.print(mTemp + " *C");
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

  //see TSL2572 datasheet: https://www.mouser.com/ds/2/588/TSL2672_Datasheet_EN_v1-255964.pdf
  cpl = 51.87 * (float)gain_val / 60.0;
  if (GAIN_DIVIDE_6) cpl /= 6.0;
  lux1 = ((float)c0 - (1.87 * (float)c1)) / cpl;
  lux2 = ((0.63 * (float)c0) - (float)c1) / cpl;
  cpl = max(lux1, lux2);
  return max(cpl, 0.0);
}
