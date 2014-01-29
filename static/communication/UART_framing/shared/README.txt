License : 
    TODO
    Written by Quentin VEY on the 2014/01/28


Presentation :
    This library is designed to send frames over the UART (instead of using it as the traditionnal 
    bytestream). Although it was first written to communicate with an Xbee in trough the API mode 2, 
    this library can be used to transmit arbitrary frames (chunks of data).

Technical details :
    The structure of the frame is the following :

                    |       |      |      |                        |        |
                    | 0x7E  | size | size | ........payload....... |checksum|
                    |(start)| (MSB)| (LSB)|                        |        |
                      bit 1   bit2   bit 3       bits 4 to n-1       bit n 

    With size<=128 and size==n-4.

    Any byte (except the first) of a frame is escaped (XORed with 0x20) if they match certain values.
    (See lib_UART_framing.h for detailed values)

    The following actions are done before accepting an incoming frame :
        * Check if it starts with start byte (0x7E).
        * Check if size is exactly the size written in bytes 2 and 3.
        * Verifies if checksum is correct (applies on payload).
        * Unscapes any previously escaped character.

Requirements :
    Requires to define the following symbols :
	    ARCH_XXX  (currently supported XXX : 328P_ARDUINO & X86_LINUX)
	    ARCH_BIG_ENDIAN xor ARCH_LITTLE_ENDIAN 
	
    Requires a visibility on a "node_cfg.h" file defining :

        #define UART_WAITFRAME_TIMEOUT xxx       //in µs
        #define UART_READBYTE_TIMEOUT  xxx       //in µs

    Requires other libraries to be located at the same place as in the ENAC robotic club git 
    repository.

    Requires that Timeout library is also correctly build.

Usage : 
    UART_init  : must be called first (for arguments, see specific header files in architecture folders).
    UART_writeFrame : to send a payload within a frame.
    UART_readFrame : to read a whole frame and return the payload.

