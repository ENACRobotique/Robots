#define PIN_BOUTON 2
#include <Wire.h>
#include <Servo.h>
#include "Adafruit_TCS34725.h"

// Pick analog outputs, for the UNO these three work well
// use ~560  ohm resistor between Red & Blue, ~1K for green (its brighter)
#define redpin 3
#define greenpin 5
#define bluepin 6
// for a common anode LED, connect the common pin to +5V
// for common cathode, connect the common to ground

// set to false if using a common cathode LED
#define commonAnode true

// our RGB -> eye-recognized gamma color
byte gammatable[256];
#define NB_COULEUR 9
float tab_couleur[NB_COULEUR][4]={
  {  3701  ,  65.16,   91.10 , 104.03 },
  {  12935 ,  59.02 ,  85.85 , 114.85 },
  {  21504 ,  89.05 , 104.62 ,  65.68 },
  {  9678  , 120.04 ,  65.89 ,  79.28 },
  {  17775 ,  50.09 ,  96.12 , 113.40 },
  {  15338 , 115.27 ,  79.88 ,  67.96 },
  {  10448 ,  94.31 ,  90.44 ,  77.82 },
  {  13021 ,  80.06 ,  77.66 , 102.20 },
  {  14661 ,  69.85 , 109.62 ,  78.94 }
};
char* tab_coul_name[NB_COULEUR]={"RIEN","Violet","Jaune","Rouge","Bleu","Orange","Marron","Rose","Vert"};

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

Servo cerveau;

void setup() {
  pinMode(PIN_BOUTON,INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("Color View Test!");

  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
  
  // use these three pins to drive an LED
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  //use the servo to turn to the right angle
  cerveau.attach(4);
  cerveau.write(150); 
  delay(3000);
  cerveau.write(179);
  delay(350);
  cerveau.write(150);
  
  // thanks PhilB for this gamma table!
  // it helps convert RGB colors to what humans see
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;
      
    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;      
    }
    //Serial.println(gammatable[i]);
  }
}


void loop() {
  uint16_t clear, red, green, blue;
  int min_ind=0;

  tcs.setInterrupt(false);      // turn on LED

  delay(60);  // takes 50ms to read 
  
  tcs.getRawData(&red, &green, &blue, &clear);

  tcs.setInterrupt(true);  // turn off LED
  // Figure out some basic hex code for visualization
  uint32_t sum = clear;
  float r, g, b;
  r = red; r /= sum;
  g = green; g /= sum;
  b = blue; b /= sum;
  r *= 256; g *= 256; b *= 256;

  //if(!digitalRead(PIN_BOUTON)){
    Serial.print("C:\t"); Serial.print(clear);
    Serial.print("\tR:\t"); Serial.print(r);
    Serial.print("\tG:\t"); Serial.print(g);
    Serial.print("\tB:\t"); Serial.print(b);

    float mini=-1;
    for(int i=0;i<NB_COULEUR;i++){
      float dist=//pow(sum-tab_couleur[i][0],2)+
                 pow(r - tab_couleur[i][1],2)+
                 pow(g - tab_couleur[i][2],2)+
                 pow(b - tab_couleur[i][3],2);
      if(dist<mini || mini<0){
        mini=dist;
        min_ind=i;
      }
    }    
    Serial.print("\tRES:\t"); Serial.println(tab_coul_name[min_ind]);
    delay(500);
  //}
  //Serial.print((int)r ); Serial.print(" "); Serial.print((int)g);Serial.print(" ");  Serial.println((int)b );

  analogWrite(redpin, gammatable[(int)r]);
  analogWrite(greenpin, gammatable[(int)g]);
  analogWrite(bluepin, gammatable[(int)b]);

  cerveau.write(min_ind*15);
  delay(3000);

  cerveau.write(150);
  delay(1000);
  cerveau.write(179);
  delay(150);
  cerveau.write(150);
}

