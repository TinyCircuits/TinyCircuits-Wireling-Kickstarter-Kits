/*template < int SIZE >
  class runningAverage
  {
    float array [ SIZE ];
  };

  runningAverage < 10 > ra;
*/
//template < int SIZE >

#include "Arduino.h"
//#include "font.h"


#define colorDepth8BPP 8
#define colorDepth16BPP 16
#define colorDepth1BPP 1

#ifndef TinyScreen_h
typedef struct
{
	const uint8_t width;
	const uint16_t offset;
	
} FONT_CHAR_INFO;	

typedef struct
{
	const unsigned char height;
	const char startCh;
	const char endCh;
	const FONT_CHAR_INFO*	charDesc;
	const unsigned char* bitmap;
		
} FONT_INFO;	
#endif

//template < const uint16_t SIZE >
class TinyBuffer : public Print {
  public:
    //init, control
    TinyBuffer(uint8_t width, uint8_t height, uint8_t colorDepth);
    //basic graphics commands
    void writePixel(uint16_t);
    //void writeBuffer(uint8_t *, int);
    void setX(uint8_t, uint8_t);
    void setY(uint8_t, uint8_t);
    void goTo(uint8_t x, uint8_t y);
    void drawPixel(uint8_t, uint8_t, uint16_t);
    void clear();
    //font
    void setFont(const FONT_INFO&);
    uint8_t getFontHeight(const FONT_INFO&);
    uint8_t getFontHeight(void);
    uint8_t getPrintWidth(char *);
    void setCursor(int, int);
    void fontColor(uint16_t, uint16_t);
    virtual size_t write(uint8_t);
    uint8_t* getBuffer();
    uint16_t getBufferSize();

  private:
    uint8_t* bufferData;
    uint16_t  _bufferSize;
    int _cursorX, _cursorY;
    uint8_t _xMax, _yMax, _pixelXinc, _pixelYinc, _cursorXmin, _cursorYmin, _cursorXmax, _cursorYmax, _fontHeight, _fontFirstCh, _fontLastCh, _bitDepth, _colorMode, _type;
    uint16_t  _fontColor, _fontBGcolor;
    const FONT_CHAR_INFO* _fontDescriptor;
    const unsigned char* _fontBitmap;
};

//struct tinyBufferBuff

//#define TinyBuffer(x,y,z) TinyBuffer(x,y,z);uint8_t TinyBufferBuffer[x*y];
