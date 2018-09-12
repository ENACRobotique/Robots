#include "sprit.h"

//Useless function defined in libstdc++ (not accessible by avr-gcc)
void __cxa_pure_virtual() { while (1); }

RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false,32);

void init_matrix(){
  matrix.begin();  
}

void erase_all(){
  int allocsize = 32 * 4 * 3 *2;
  memset(matrix.matrixbuff [0], 0, allocsize);
}
void drawPixel2(int16_t x, int16_t y, uint8_t r,uint8_t g,uint8_t b,int8_t offset) {
  if(x<0 || x>=32 || y<0 || y>=16)return;
  
  x+=offset+32;x%=32;
  
  //manage color
  r&=0xF;g&=0xF;b&=0xF;//Max value
  uint8_t c0=(r&0x1)<<0 | (g&0x1)<<1 | (b&0x1)<<2;
  uint8_t c1=(r&0x2)>>1 | (g&0x2)<<0 | (b&0x2)<<1;
  uint8_t c2=(r&0x4)>>2 | (g&0x4)>>1 | (b&0x4)<<0;
  uint8_t c3=(r&0x8)>>3 | (g&0x8)>>2 | (b&0x8)>>1;
  
  //manage x
  int16_t real_x=x%16;
  if( (x%16)/4 >0 )real_x+=8;
  if( (x%16)/4 >2 )real_x+=8;
  if( (x/16)   >0 )real_x+=32;
  //manage y
  if(  y   >=8){
    c0<<=3;
    c1<<=3;
    c2<<=3;
    c3<<=3;
  }
  if( (y%8)>=4){
    if( (x/4)%2==1)real_x-=4;
    else real_x+=4;
  }
  //planes 1-3
  matrix.matrixbuff[0][real_x+64*3*(y%4)+64*0]|=c1<<2;
  matrix.matrixbuff[0][real_x+64*3*(y%4)+64*1]|=c2<<2;
  matrix.matrixbuff[0][real_x+64*3*(y%4)+64*2]|=c3<<2;
  //plane 0
  matrix.matrixbuff[0][real_x+64*3*(y%4)+64*0]|=(c0>>4)&0x3;
  matrix.matrixbuff[0][real_x+64*3*(y%4)+64*1]|=(c0>>2)&0x3;
  matrix.matrixbuff[0][real_x+64*3*(y%4)+64*2]|=(c0>>0)&0x3;
  
}

//void matrix
