/************************************************************************
 * Wireling Basic Kit Shipment Program
 * This program uses all six of the Wirelings included with the Basic Kit:
 * Port 0: 0.42" OLED Screen Wireling
 * Port 1: Ambient Light Sensor Wireling
 * Port 2: Digital Sensor Wireling OR Large Button Wireling
 * Port 3: RGB LED Wireling OR Buzzer Wireling
 * 
 * When plugged in according to the above mapping, the 0.42" Screen will 
 * display the lux value read from the light sensor, and will also display 
 * the HIGH or LOW state of whatever Wireling is connected to Port 2 (The 
 * digital hall, or large button). When the Wireling on Port 2 reads LOW, 
 * the RGB LED or Buzzer connected to Port 3 will either light up a short 
 * RGB sequence, or play a few notes, respectively. 
 *
 * Hardware by: TinyCircuits
 * Written by: Laveréna Wienclaw for TinyCircuits
 *
 * Initiated: 11/2/2019 
 * Updated: 12/5/2019
 ************************************************************************/

#include <Wire.h>                   // For I2C communication
#include <Wireling.h>               // For interfacing with Wirelings
#include "Font_042.h"               // The font displayed on the screen
#include "TinyCircuits_HP7240.h"    // Library for OLED screen
#include "exampleSprites.h"         // Holds arrays of example Sprites
#include "pitches.h"                // Tones used with the Buzzer
#include <FastLED.h>                // For the RGB LED


/***************************** 042 Screen OLED Varibles ****************************/                                 
TinyCircuits_HP7240 TiniestScreen;

#define xMax TiniestScreen.xMax
#define yMax TiniestScreen.yMax

uint8_t oledbuf[HP7240_BUFFERSIZE]; // Buffer to hold screen data

int bufIndex = 0; // Buffer index in respect to the array of pixels on the screen
int textPos = 0;  // Used to make text move left or right
/***********************************************************************************/


/************************* Ambient Light Sensor Variables **************************/
// Communication address with the sensor
#define TSL2572_I2CADDR     0x39

// Sets the gain
#define   GAIN_1X 0
#define   GAIN_8X 1
#define  GAIN_16X 2
#define GAIN_120X 3

//only use this with 1x and 8x gain settings
#define GAIN_DIVIDE_6 true

// Global variable for gain value used to Read the sensor
int gain_val = 0;
/***********************************************************************************/


/***************************** Buzzer Sensor Variables *****************************/
// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};
/***********************************************************************************/


/******************************** RGB LED Variables ********************************/
#define NUM_LEDS 1 // This is the number of RGB LEDs connected to the pin
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
int brightness = 128; // Brightness is on a scale of 0-255, 128 is 50% brightness
/***********************************************************************************/


/*********************************** Wireling Variables ****************************/
#define SCREEN_PORT 0   
#define RESET_PIN (uint8_t)A0       // A0 corresponds to Port 0. Update if changing ports for 042 Screen.
                                    // Port1: A1, Port2: A2, Port3: A3             
#define AMBIENT_PORT 1
#define BUTTON_HALL_PIN (uint8_t)A2
#define BUZZER_RGB_PIN (uint8_t) A3

bool highOrLow = false;
/***********************************************************************************/


void setup() {  
  SerialUSB.begin(9600);

  // Enable & Power Wirelings
  Wireling.begin();

  // Initialize 0.42" OLED Screen
  initScreen();

  // Initialize Ambient Light Sensor
  Wireling.selectPort(AMBIENT_PORT);
  TSL2572Init(GAIN_16X);

  // Initialize RGB LED
  FastLED.addLeds<WS2812, BUZZER_RGB_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(brightness);
  pinMode(BUZZER_RGB_PIN, OUTPUT);
}

void loop() {

  /************************************ Port 0 ************************************/
  Wireling.selectPort(SCREEN_PORT);
  clearOLED();
  
  /************************************ Port 1 ************************************/
  Wireling.selectPort(AMBIENT_PORT);
  float Lux = Tsl2572ReadAmbientLight();

  // Create a char array with results to print to screen
  String luxStrVal = String(Lux);
  String luxFullString = ("Lux:" + luxStrVal);
  char luxBuf[10];
  luxFullString.toCharArray(luxBuf, 10);
  
  Wireling.selectPort(SCREEN_PORT);
  textPos = 3*72;
  TiniestScreen.setCursorX(textPos);
  TiniestScreen.printSSD(oledbuf, luxBuf); 
  
  /************************************ Port 2 ************************************/
  highOrLow = digitalRead(BUTTON_HALL_PIN);
  Wireling.selectPort(SCREEN_PORT);
  textPos = 1*72;
  TiniestScreen.setCursorX(textPos);
  
  if(highOrLow){
    TiniestScreen.printSSD(oledbuf, "Port 2: HIGH"); 
  }
  else if (!highOrLow){
    TiniestScreen.printSSD(oledbuf, "Port 2: LOW"); 
  }
  else{
    TiniestScreen.printSSD(oledbuf, "Port 2: N/A"); 
  }
  TiniestScreen.sendFramebuffer(oledbuf);
  
  /************************************ Port 3 ************************************/
  if(!highOrLow) // button was pressed, or hall was activated
  {
      for(int x = 0; x < NUM_LEDS; x++) // cycle through all LEDs attached to LED_PIN
      {
        leds[x] = CRGB( 255, 0, 0); // RED
        FastLED.show();
        delay(100);
        leds[x] = CRGB(0, 255, 0); // GREEN
        FastLED.show();
        delay(100);
        leds[x] = CRGB(0, 0, 255); // BLUE
        FastLED.show();
        delay(100);
        leds[x] = CRGB(0, 0, 0); // NOTHING
        FastLED.show();
        delay(5);
      }
      
      for (int thisNote = 0; thisNote < 8; thisNote++) {
        // to calculate the note duration, take one second divided by the note type.
        //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
        int noteDuration = 1000 / noteDurations[thisNote];
        tone(BUZZER_RGB_PIN, melody[thisNote], noteDuration);
    
        // to distinguish the notes, set a minimum time between them.
        // the note's duration + 30% seems to work well:
        int pauseBetweenNotes = noteDuration * 1.30;
        delay(pauseBetweenNotes);
        // stop the tone playing:
        noTone(8);
      }   
  }
}

/********************************************* 042 Screen Functions ********************************/
void initScreen(void)
{
  TiniestScreen.begin();    // begin I2C communications with screen
  Wireling.selectPort(SCREEN_PORT);  // This port# matches the one labeled on the adapter board
  TiniestScreen.resetScreen(RESET_PIN);   // resets whisker screen MUST BE CALLED BEFORE init()
  TiniestScreen.init();     // initialize screen
  clearOLED();              // Clear Display Buffer, isn't fully cleared here, frame buffer must be sent
  delay(2);
  TiniestScreen.sendFramebuffer(oledbuf); // Send Cleared Buffer
}

// Writes blank data to OLED to clear data
void clearOLED() {
  for (int i = 0; i < HP7240_BUFFERSIZE; i++) {
    oledbuf[i] = 0x00;
  }
}

void setPixel(int px, int py) {
  int pos = px;       //holds the given X-coordinate
  if (py > 7) { // if Y > 7 (the number of indices in a byte)
    pos += (py/8)*xMax; // bump down to the next row by increasing X by the screen width by the number of necessary rows
  }
  py = (py % 8);  // adjusts Y such that it can be written within the 0-7 bounds of a byte
  oledbuf[pos] |= (1 << (py)); // the bits of the byte within the buffer are set accordingly by placing a 1 in the respective bit location of the byte
}
/**********************************************************************************************/


/************************************* Ambient Light Functions **********************************/
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

  //see TSL2572 datasheet: https://www.mouser.com/ds/2/588/TSL2672_Datasheet_EN_v1-255424.pdf
  cpl = 51.87 * (float)gain_val / 60.0;
  if (GAIN_DIVIDE_6) cpl /= 6.0;
  lux1 = ((float)c0 - (1.87 * (float)c1)) / cpl;
  lux2 = ((0.63 * (float)c0) - (float)c1) / cpl;
  cpl = max(lux1, lux2);
  return max(cpl, 0.0);
}
/**********************************************************************************************/
