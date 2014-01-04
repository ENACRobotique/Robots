Library to send frame over the UART (instead of using it as the traditionnal bytestream).

Requires to define the following symbols :
	ARCH_XXX  (currently supported : 328P_ARDUINO & X86_LINUX)
	ARCH_BIG_ENDIAN or ARCH_LITTLE_ENDIAN 
	
Requires a visibility on a "node_cfg.h" file defining :

#define UART_WAITFRAME_TIMEOUT xxx       //in µs
#define UART_READBYTE_TIMEOUT  xxx       //in µs

Requires a visibility on timeout.h (like in static/core/arduino/libraries/Timeout/)