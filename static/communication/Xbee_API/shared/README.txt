Library to use the XBEE API mode 2.
Does not implement all the possibilities of zigbee protocol.

Requires to define the following symbols :
	ARCH_XXX  (currently supported : 328P_ARDUINO & X86_LINUX)
	ARCH_BIG_ENDIAN or ARCH_LITTLE_ENDIAN 
	
Requires compilation of the UART_framing library

Requires a visibility on timeout.h (like in static/core/arduino/libraries/Timeout/)