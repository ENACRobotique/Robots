#include "Arduino.h"
#include "wiring.c"
int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

