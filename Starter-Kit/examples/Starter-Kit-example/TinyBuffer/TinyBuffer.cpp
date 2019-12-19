#include "TinyBuffer.h"
#include <avr/pgmspace.h>
//template < const uint16_t SIZE >
TinyBuffer::TinyBuffer(uint8_t width, uint8_t height, uint8_t colorDepth) {
  _xMax = width-1;
  _yMax = height-1;
  _cursorX = 0;
  _cursorY = 0;
  _cursorXmin = 0;
  _cursorYmin = 0;
  _cursorXmax = 0;
  _cursorYmax = 0;
  _fontHeight = 0;
  //_fontWidth = 0;??????
  _fontFirstCh = 0;
  _fontLastCh = 0;
  _fontDescriptor=0;
  _fontBitmap=0;
  _fontColor=0xFFFF;
  _fontBGcolor=0x0000;
  _bitDepth=colorDepth;
  _colorMode = 0;
  _bufferSize = width*height;
  if(_bitDepth==16)_bufferSize*=2;
  if(_bitDepth==1)_bufferSize/=8;
  bufferData=0;
  bufferData=(uint8_t *)malloc(_bufferSize);
  if(!bufferData)while(1);
}

//template < const uint16_t SIZE >
//size_t TinyBuffer::write(uint8_t ch){
//  return 1;
//}


void TinyBuffer::goTo(uint8_t x, uint8_t y) {
  if(x>_xMax||y>_yMax)return;
  setX(x,_xMax);
  setY(y,_yMax);
}

void TinyBuffer::setX(uint8_t x, uint8_t end) {
  if(x>_xMax)x=_xMax;
  if(end>_xMax)end=_xMax;
  _cursorX = x;
  _pixelXinc = x;
  _cursorXmin = x;
  _cursorXmax = end;
}

void TinyBuffer::setY(uint8_t y, uint8_t end) {
  if(y>_yMax)y=_yMax;
  if(end>_yMax)end=_yMax;
  _cursorY = y;
  _pixelYinc = y;
  _cursorYmin = y;
  _cursorYmax = end;
}

void TinyBuffer::clear(){
  memset(bufferData, 0x00, _bufferSize);
}

uint8_t* TinyBuffer::getBuffer(){
  return bufferData;
}

uint16_t TinyBuffer::getBufferSize(){
  return _bufferSize;
}

void TinyBuffer::drawPixel(uint8_t x, uint8_t y, uint16_t color)
{
  if(x>_xMax||y>_yMax)return;
  goTo(x,y);
  writePixel(color);
}

void TinyBuffer::writePixel(uint16_t color) {
  uint16_t offset = (uint16_t)(_pixelYinc*(_xMax+1))+(uint16_t)_pixelXinc;
  if(_bitDepth==16){
    bufferData[offset*2] =  color >> 8;
    bufferData[offset*2+1] =  color;
  }else if(_bitDepth==16){
    bufferData[offset] =  color;
  }else if(_bitDepth==1){
    //(_xMax+1)???
    uint16_t pos = _pixelXinc;               //holds the given X-coordinate
    if (_pixelYinc > 7) {               // if Y > 7 (the number of indices in a byte)
      pos += (_pixelYinc/8) * (_xMax+1);    // bump down to the next row by increasing X by the screen width by the number of necessary rows
    }
    uint8_t bitNum = (_pixelYinc % 8);              // adjusts Y such that it can be written within the 0-7 bounds of a byte
    //buffer[pos] |= (1 << (py)); // the bits of the byte within the buffer are set accordingly by placing a 1 in the respective bit location of the byte
    if(color){
      bufferData[pos] |=  (1 << (bitNum));
    }else{
      bufferData[pos] &=  ~(1 << (bitNum));
    }
  }
  
  _pixelXinc++;
  if(_pixelXinc>_cursorXmax){
    _pixelXinc = _cursorXmin;
    _pixelYinc++;
    if(_pixelYinc>=_cursorYmax){
      _pixelYinc=_cursorYmin;
    }
  }
}

void TinyBuffer::setCursor(int x, int y){
  _cursorX=x;
  _cursorY=y;
}

void TinyBuffer::setFont(const FONT_INFO& fontInfo){
  _fontHeight=fontInfo.height;
  _fontFirstCh=fontInfo.startCh;
  _fontLastCh=fontInfo.endCh;
  _fontDescriptor=fontInfo.charDesc;
  _fontBitmap=fontInfo.bitmap;
}

void TinyBuffer::fontColor(uint16_t f, uint16_t g){
  _fontColor=f;
  _fontBGcolor=g;
}

uint8_t TinyBuffer::getFontHeight(const FONT_INFO& fontInfo){
  return fontInfo.height;
}

uint8_t TinyBuffer::getFontHeight(){
  return _fontHeight;
}

uint8_t TinyBuffer::getPrintWidth(char * st){
  if(!_fontFirstCh)return 0;
  uint8_t i,amtCh,totalWidth;
  totalWidth=0;
  amtCh=strlen(st);
  for(i=0;i<amtCh;i++){
    totalWidth+=pgm_read_byte(&_fontDescriptor[st[i]-_fontFirstCh].width)+1;
  }
  return totalWidth;
}

size_t TinyBuffer::write(uint8_t ch){
  if(!_fontFirstCh)return 1;
  if(ch<_fontFirstCh || ch>_fontLastCh)return 1;
  if(_cursorX>_xMax || _cursorY>_yMax)return 1;
  uint8_t chWidth=pgm_read_byte(&_fontDescriptor[ch-_fontFirstCh].width);
  
  
  uint8_t ySkip=0;
  if(_cursorY<0){
    if(_cursorY+_fontHeight<0){
      return 1;
    }
    ySkip=0-_cursorY;
  }
  
  uint8_t xSkip=0;
  uint8_t byteSkip=0;
  if(_cursorX<0){
    if(_cursorX+chWidth<0){
     _cursorX+=(int)(chWidth+1);
      return 1;
    }
    xSkip=0-_cursorX;
    //todo
    //byteSkip=xSkip/8;
    //if(byteSkip){
    //  xSkip=
    //}
  }
  uint8_t bytesPerRow=chWidth/8;
  if(chWidth>bytesPerRow*8)
    bytesPerRow++;
  uint16_t offset=pgm_read_word(&_fontDescriptor[ch-_fontFirstCh].offset)+(bytesPerRow*_fontHeight)-1;
  
  setX(_cursorX+xSkip,_cursorX+chWidth);//had chWidth+1, but not using preceding fontbg column?
  setY(_cursorY+ySkip,_cursorY+_fontHeight);
  
  //startData();
  for(uint8_t y=ySkip; y<_fontHeight && ((y+_cursorY)<(_yMax)); y++){//////was (_yMax+1)
    //writePixel(_fontBGcolor);//
    for(uint8_t byte=0; byte<bytesPerRow; byte++){
      uint8_t data=pgm_read_byte(_fontBitmap+offset-y-((bytesPerRow-byte-1)*_fontHeight));
      uint8_t bits=byte*8;
        for(uint8_t i=xSkip; i<8 && (bits+i)<chWidth && (bits+i+_cursorX)<(_xMax+1); i++){//was (_xMax)
          if(data&(0x80>>i)){
            writePixel(_fontColor);
           }else{
            writePixel(_fontBGcolor);
          }
      }
    }
    if((_cursorX+chWidth)<=_xMax){//did not have =
      writePixel(_fontBGcolor);
    }
  }
  //endTransfer();
  /*if(_cursorX<0){
    SerialUSB.print(_cursorX);
    SerialUSB.print(" ");
  _cursorX+=(int)(chWidth+1);
    SerialUSB.println(_cursorX);
  }*/
  _cursorY-=ySkip;//restore _cursorY
  _cursorX+=(int)(chWidth+1-xSkip);//needs -xSkip due to running setX which will 0 _cursorX
  return 1;
}