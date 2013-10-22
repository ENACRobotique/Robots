#include <lpc214x.h>

#include <gpio.h>
#include <sys_time.h>

int	main() {
	unsigned int prevMillis = 0;
	unsigned int time;
	int state = 0;

	gpio_init_all();
	sys_time_init();

	gpio_output(1, 24);

	while(1) {
		sys_time_update();

		time = millis();

		if( (time - prevMillis) > 1000 ) {
			prevMillis = time;
			gpio_write(1, 24, state^=1);
		}
	}
}
