#include "sprit.h"

#include "image.h"

void setup() {
  init_matrix();
  draw_image(0);
}

void loop() {
  for(int i=0;;i--){
    erase_all();
    draw_image(i%32);
    delay(1000);
  }
  
}
