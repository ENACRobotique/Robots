
// testshapes demo for Adafruit RGBmatrixPanel library.
// Demonstrates the drawing abilities of the RGBmatrixPanel library.
// For 16x32 RGB LED matrix:
// http://www.adafruit.com/products/420

// Written by Limor Fried/Ladyada & Phil Burgess/PaintYourDragon
// for Adafruit Industries.
// BSD license, all text above must be included in any redistribution.

#include <SPI.h>
#include "Adafruit_GFX.h"
#include "RGBmatrixPanel.h"

#define CLK 8  // MUST be on PORTB! (Use pin 11 on Mega)
#define LAT A3
#define OE  9
#define A   A0
#define B   A1
#define C   A2
#define D   A3
RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, false);
//RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false,32);

#define OFFSET_COTE 32*8*3


#define SCORE_COLOR matrix.Color333(0, 1, 3)
#define DEMARQ_COLOR matrix.Color333(1, 0, 0)
const int score=109;


void plot_A(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}
void plot_B(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}
void plot_C(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color);  
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,7,color); 
}
void plot_D(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}
void plot_E(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,7,color); 
}
void plot_I(int offset_x,uint16_t color=SCORE_COLOR){
  //First row 
  matrix.drawPixel(1+offset_x,0,3,color); 
  matrix.drawPixel(2+offset_x,0,3,color); 
  //Second row
  matrix.drawPixel(1+offset_x,1,3,color); 
  matrix.drawPixel(2+offset_x,1,3,color); 
  //Thrird row
  matrix.drawPixel(1+offset_x,2,0,color); 
  matrix.drawPixel(2+offset_x,2,0,color); 
}
void plot_L(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,7,color); 
}
void plot_N(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  //Second row
  matrix.drawPixel(0+offset_x,1,7,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,7,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}

void plot_S(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(1+offset_x,1,3,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,6,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,3,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,6,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}
void plot_U(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}
void plot_V(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(1+offset_x,2,0,color); 
  matrix.drawPixel(2+offset_x,2,0,color); 
}
void plot_apostrophe(int offset_x,uint16_t color=SCORE_COLOR){
  //First row 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,3,color); 
}
void plot_2point(int offset_x,uint16_t color=SCORE_COLOR){
  //First row 
  matrix.drawPixel(1+offset_x,1,3,color); 
  matrix.drawPixel(2+offset_x,1,3,color); 
  matrix.drawPixel(1+offset_x,2,3,color); 
  matrix.drawPixel(2+offset_x,2,3,color); 
}
void plot_exclamation(int offset_x,uint16_t color=SCORE_COLOR){
  //First row 
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  //Second row 
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(2+offset_x,1,0,color); 
  //Thrid row 
  matrix.drawPixel(0+offset_x,2,4,color); 
  matrix.drawPixel(2+offset_x,2,4,color); 
}

void plot_0(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}
void plot_1(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
}

void plot_2(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,7,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,7,color); 
}

void plot_3(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,7,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,7,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}


void plot_4(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(3+offset_x,2,0,color); 
}

void plot_5(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,7,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,7,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}

void plot_6(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,7,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}

void plot_7(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(3+offset_x,2,0,color); 
}

void plot_8(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,0,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}

void plot_9(int offset_x,uint16_t color=SCORE_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,7,color); 
  matrix.drawPixel(1+offset_x,0,7,color); 
  matrix.drawPixel(2+offset_x,0,7,color); 
  matrix.drawPixel(3+offset_x,0,7,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  matrix.drawPixel(1+offset_x,1,7,color); 
  matrix.drawPixel(2+offset_x,1,7,color); 
  matrix.drawPixel(3+offset_x,1,0,color); 
  //Thrird row
  matrix.drawPixel(0+offset_x,2,7,color); 
  matrix.drawPixel(1+offset_x,2,7,color); 
  matrix.drawPixel(2+offset_x,2,7,color); 
  matrix.drawPixel(3+offset_x,2,0,color); 
}

void plot(int nb,int offset_x,uint16_t color=SCORE_COLOR){
  switch(nb){
    case 0:case '0': plot_0(offset_x,color);break;
    case 1:case '1': plot_1(offset_x,color);break;
    case 2:case '2': plot_2(offset_x,color);break;
    case 3:case '3': plot_3(offset_x,color);break;
    case 4:case '4': plot_4(offset_x,color);break;
    case 5:case '5': plot_5(offset_x,color);break;
    case 6:case '6': plot_6(offset_x,color);break;
    case 7:case '7': plot_7(offset_x,color);break;
    case 8:case '8': plot_8(offset_x,color);break;
    case 9:case '9': plot_9(offset_x,color);break;
    case 'A':case 'a': plot_A(offset_x,color);break;
    case 'C':case 'c': plot_C(offset_x,color);break;
    case 'D':case 'd': plot_D(offset_x,color);break;
    case 'E':case 'e': plot_E(offset_x,color);break;
    case 'I':case 'i': plot_I(offset_x,color);break;
    case 'L':case 'l': plot_L(offset_x,color);break;
    case 'N':case 'n': plot_N(offset_x,color);break;
    case 'O':case 'o': plot_0(offset_x,color);break;
    case 'S':case 's': plot_S(offset_x,color);break;
    case 'U':case 'u': plot_U(offset_x,color);break;
    case 'V':case 'v': plot_V(offset_x,color);break;
    case '\'':plot_apostrophe(offset_x,color);break;
    case '!':plot_exclamation(offset_x,color);break;
    case ':':plot_2point(offset_x,color);break;
  }
}
void plot(char tab[],int8_t offset_x,uint16_t color=SCORE_COLOR){
  for(int i=0;tab[i]!='\0';i++){//TODO change this shit!
    plot(tab[i],(offset_x+5*i),color);
  }
}
void plot_nb(int nb,uint8_t offset_n,uint16_t color=SCORE_COLOR){
  int delta_nb=5;
  if(nb>0)
    plot(nb%10,offset_n-delta_nb+1,color);
  if(nb<10)return;
  return plot_nb(nb/10,offset_n-delta_nb,color);
}
void plot_demarquation(int offset_x=0,uint16_t color=DEMARQ_COLOR){
  //First row
  matrix.drawPixel(0+offset_x,0,0,color); 
  //Second row
  matrix.drawPixel(0+offset_x,1,0,color); 
  //Third row
  matrix.drawPixel(0+offset_x,2,0,color); 
  //Last row
  matrix.drawPixel(0+offset_x,3,0,color); 
}
void erase_all(){
  int allocsize = 32 * 8 * 3 *2;
  memset(matrix.matrixbuff [0], 0, allocsize);
}

void setup() {
  matrix.begin();
}

void loop() {
  
  for(int i=0;;i++){
    erase_all();
    plot("envoie LOVE: 06 35 96 75 90",32-i%(32+27*5),matrix.Color333(2, 0, 1));
    //plot("ENAC ENAC ON S'ENCULE!!",32-i%(32+23*5));
    delay(100);
  }
  /*
  for(int i=0;;i++){
   erase_all();
   plot_nb(i,32);
   delay(10); 
  }*/
  
  //matrix.drawPixel2(i,j,0, SCORE_COLOR);
  for(;;)delay(1000);
  
  
  
  uint16_t color_count=matrix.Color333(3, 3, 0);
  uint16_t color_cligno=matrix.Color333(1,1,1);
  uint16_t color_final=SCORE_COLOR;

  //on compte !
  int shift=32;
  for(int i=0;i<=score;i++){
    erase_all();
    plot( "ENAC",1);
    plot_nb( i,shift,color_count);
    if(i==42)delay(1000);
    else if(i==31)delay(1500);
    else if(i==57)delay(500);
    else if(i==69)delay(500);
    else delay(50);
  }
  //cligno epileptique !!!
  for(int i=0;i<10;i++){
    erase_all();
    plot( "ENAC",1);
    plot_nb( score,shift,color_cligno);
    delay(50);
    erase_all();
    plot( "ENAC",1);
    plot_nb( score,shift,color_final);
    delay(50);
  }
  for(;;){delay(1000);}
}
