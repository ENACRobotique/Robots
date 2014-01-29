License : 
    TODO
    Written by quentin VEY on the 2014/01/29


Presentation :
    Library created to help the use of XBEE API mode 2. Every define is done, but this library does 
    not implement all the possibile action in zigbee protocol.

Technical details :
    Refer to digi's official documentation for details.

Requirements :
    Requires UART_framing library to be build and linked (refer to this latter for details).
    Requires that Timeout library is also correctly build and linked.

    Requires to define the following symbols :
	    ARCH_XXX  (currently supported : 328P_ARDUINO & X86_LINUX)
	    ARCH_BIG_ENDIAN xor ARCH_LITTLE_ENDIAN 
	
    Requires a visibility on a "node_cfg.h" file defining :

        #define XBEE_RST_PIN                    // for arduino only
        #define XBEE_UART_PATH "/dev/ttyXXXX"   // for linux only

    Requires other libraries to be located at the same place as in the ENAC robotic club git 
    repository.


Usage : 
    refer to .c file for details.

