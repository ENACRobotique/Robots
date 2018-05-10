// testshapes demo for Adafruit RGBmatrixPanel library.
// Demonstrates the drawing abilities of the RGBmatrixPanel library.
// For 16x32 RGB LED matrix:
// http://www.adafruit.com/products/420

// Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon
// for Adafruit Industries.
// BSD license, all text above must be included in any redistribution.

#include "includer.h"



#define CLK 8  // MUST be on PORTB! (Use pin 11 on Mega)
#define LAT A3
#define OE  9
#define A   A0
#define B   A1
#define C   A2
RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, false);

#define OFFSET_CENTAINE -1
#define OFFSET_DIZAINE   0
#define OFFSET_UNITE     1

#define SCORE_COLOR matrix.Color333(0, 1, 3)
#define DEMARQ_COLOR matrix.Color333(1, 0, 0)
const int score=109;

void calc_offset(int offset,int &off1,int &off2){
  switch(offset){
    case OFFSET_UNITE:
      off1=8+1;
      off2=12+1;
      return;
    case OFFSET_DIZAINE:
      off1=4;
      off2=0;
      return;
    case OFFSET_CENTAINE:
      off1=-9;
      off2=-5;
      return;
  }
}

void plot_0(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  //First row
  matrix.drawPixel( 3+off2,7  , color); 
  matrix.drawPixel( 8+off1,7  , color); 
  matrix.drawPixel( 9+off1,7  , color); 
  matrix.drawPixel(10+off1,7  , color); 
  //Second row
  matrix.drawPixel( 7+off2,0  , color); 
  matrix.drawPixel(10+off2,0  , color); 
  //Thrird row
  matrix.drawPixel( 3+off2,0+8, color); 
  matrix.drawPixel( 8+off1,7+8, color); 
  matrix.drawPixel( 9+off1,7+8, color); 
  matrix.drawPixel(10+off1,0+8, color); 
}

void plot_1(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  
  matrix.drawPixel(10+off1,7  , color); 
  matrix.drawPixel(10+off2,0  , color); 
  matrix.drawPixel(10+off1,0+8, color); 
}

void plot_2(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  //First row
  matrix.drawPixel( 3+off2,7  , color); 
  matrix.drawPixel( 8+off1,7  , color); 
  matrix.drawPixel( 9+off1,7  , color); 
  matrix.drawPixel(10+off1,7  , color); 
  //Second row
  matrix.drawPixel( 7+off2,7  , color); 
  matrix.drawPixel( 8+off2,7  , color); 
  matrix.drawPixel( 9+off2,7  , color); 
  matrix.drawPixel(10+off2,0  , color); 
  //Thrird row
  matrix.drawPixel( 3+off2,0+8, color); 
  matrix.drawPixel( 8+off1,7+8, color); 
  matrix.drawPixel( 9+off1,7+8, color); 
  matrix.drawPixel(10+off1,7+8, color); 
}

void plot_3(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  //First row
  matrix.drawPixel( 3+off2,7  , color); 
  matrix.drawPixel( 8+off1,7  , color); 
  matrix.drawPixel( 9+off1,7  , color); 
  matrix.drawPixel(10+off1,7  , color); 
  //Second row
  matrix.drawPixel( 7+off2,7  , color); 
  matrix.drawPixel( 8+off2,7  , color); 
  matrix.drawPixel( 9+off2,7  , color); 
  matrix.drawPixel(10+off2,0  , color); 
  //Thrird row
  matrix.drawPixel( 3+off2,7+8, color); 
  matrix.drawPixel( 8+off1,7+8, color); 
  matrix.drawPixel( 9+off1,7+8, color); 
  matrix.drawPixel(10+off1,0+8, color); 
}

void plot_4(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  //First row
  matrix.drawPixel( 3+off2,7  , color); 
  matrix.drawPixel(10+off1,7  , color); 
  //Second row
  matrix.drawPixel( 7+off2,0  , color); 
  matrix.drawPixel( 8+off2,7  , color); 
  matrix.drawPixel( 9+off2,7  , color); 
  matrix.drawPixel(10+off2,0  , color); 
  //Thrird row
  matrix.drawPixel(10+off1,0+8, color); 
}

void plot_5(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  //First row
  matrix.drawPixel( 3+off2,7  , color); 
  matrix.drawPixel( 8+off1,7  , color); 
  matrix.drawPixel( 9+off1,7  , color); 
  matrix.drawPixel(10+off1,7  , color); 
  //Second row
  matrix.drawPixel( 7+off2,0  , color); 
  matrix.drawPixel( 8+off2,7  , color); 
  matrix.drawPixel( 9+off2,7  , color); 
  matrix.drawPixel(10+off2,7  , color); 
  //Thrird row
  matrix.drawPixel( 3+off2,7+8, color); 
  matrix.drawPixel( 8+off1,7+8, color); 
  matrix.drawPixel( 9+off1,7+8, color); 
  matrix.drawPixel(10+off1,0+8, color); 
}

void plot_6(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  //First row
  matrix.drawPixel( 3+off2,7  , color); 
  matrix.drawPixel( 8+off1,7  , color); 
  matrix.drawPixel( 9+off1,7  , color); 
  matrix.drawPixel(10+off1,7  , color); 
  //Second row
  matrix.drawPixel( 7+off2,0  , color); 
  matrix.drawPixel( 8+off2,7  , color); 
  matrix.drawPixel( 9+off2,7  , color); 
  matrix.drawPixel(10+off2,7  , color); 
  //Thrird row
  matrix.drawPixel( 3+off2,0+8, color); 
  matrix.drawPixel( 8+off1,7+8, color); 
  matrix.drawPixel( 9+off1,7+8, color); 
  matrix.drawPixel(10+off1,0+8, color); 
}

void plot_7(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  //First row
  matrix.drawPixel( 3+off2,7  , color); 
  matrix.drawPixel( 8+off1,7  , color); 
  matrix.drawPixel( 9+off1,7  , color); 
  matrix.drawPixel(10+off1,7  , color); 
  //Second row
  matrix.drawPixel(10+off2,0  , color); 
  //Thrird row
  matrix.drawPixel(10+off1,0+8, color); 
}

void plot_8(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  //First row
  matrix.drawPixel( 3+off2,7  , color); 
  matrix.drawPixel( 8+off1,7  , color); 
  matrix.drawPixel( 9+off1,7  , color); 
  matrix.drawPixel(10+off1,7  , color); 
  //Second row
  matrix.drawPixel( 7+off2,0  , color); 
  matrix.drawPixel( 8+off2,7  , color); 
  matrix.drawPixel( 9+off2,7  , color); 
  matrix.drawPixel(10+off2,0  , color); 
  //Thrird row
  matrix.drawPixel( 3+off2,0+8, color); 
  matrix.drawPixel( 8+off1,7+8, color); 
  matrix.drawPixel( 9+off1,7+8, color); 
  matrix.drawPixel(10+off1,0+8, color); 
}

void plot_9(int offset,uint16_t color=SCORE_COLOR){
  int off1,off2;
  calc_offset(offset,off1,off2);
  //First row
  matrix.drawPixel( 3+off2,7  , color); 
  matrix.drawPixel( 8+off1,7  , color); 
  matrix.drawPixel( 9+off1,7  , color); 
  matrix.drawPixel(10+off1,7  , color); 
  //Second row
  matrix.drawPixel( 7+off2,0  , color); 
  matrix.drawPixel( 8+off2,7  , color); 
  matrix.drawPixel( 9+off2,7  , color); 
  matrix.drawPixel(10+off2,0  , color); 
  //Thrird row
  matrix.drawPixel( 3+off2,7+8, color); 
  matrix.drawPixel( 8+off1,7+8, color); 
  matrix.drawPixel( 9+off1,7+8, color); 
  matrix.drawPixel(10+off1,0+8, color); 
}
void plot(int nb,int offset,uint16_t color=SCORE_COLOR){
  switch(nb){
    case 0: plot_0(offset,color);break;
    case 1: plot_1(offset,color);break;
    case 2: plot_2(offset,color);break;
    case 3: plot_3(offset,color);break;
    case 4: plot_4(offset,color);break;
    case 5: plot_5(offset,color);break;
    case 6: plot_6(offset,color);break;
    case 7: plot_7(offset,color);break;
    case 8: plot_8(offset,color);break;
    case 9: plot_9(offset,color);break;
  }
}
void plot(int nb,uint16_t color=SCORE_COLOR){
  if(nb/100==1)
    plot(1        ,OFFSET_CENTAINE,color);
  plot((nb%100)/10,OFFSET_DIZAINE ,color);
  plot( nb%10     ,OFFSET_UNITE   ,color);
}
void plot_demarquation(uint16_t color=DEMARQ_COLOR){
  matrix.drawPixel(3+4*6, 0,   color); 
  matrix.drawPixel(3+4*7, 0,   color); 
  matrix.drawPixel(3+4*6, 0+8, color); 
  matrix.drawPixel(3+4*7, 0+8, color); 
}
void erase_all(){
  int allocsize = 32 * 16 * 3;
  memset(matrix.matrixbuff[0], 0, allocsize);
}

void setup() {
  matrix.begin();
  //SCORE
  plot(score);
  plot_demarquation();
}

void loop() {

  uint16_t color_count=matrix.Color333(3, 3, 0);
  uint16_t color_cligno=matrix.Color333(1,1,1);
  uint16_t color_final=SCORE_COLOR;

  //on compte !
  for(int i=0;i<=score;i++){
    erase_all();
    plot( i,color_count);
    plot_demarquation();
    if(i==42)delay(1000);
    else if(i==31)delay(1500);
    else if(i==69)delay(500);
    else delay(50);
  }
  //cligno epileptique !!!
  for(int i=0;i<10;i++){
    erase_all();
    plot( score,color_cligno);
    plot_demarquation();
    delay(50);
    erase_all();
    plot( score,color_final);
    plot_demarquation();
    delay(50);
  }
  
  for(;;){delay(1000);}
}
