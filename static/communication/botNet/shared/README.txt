License : 
    TODO
    Written by Qentin VEY on the 2014/01/29


Presentation :
    This library provides a lightweight network API designed to work on any single-threaded processor,
    including desktop computers, enbeeded computers and microconrolers.
    
    Currently supported architectures (also check other libraries for compatibilities, eg Xbee_API): 
        * Debian/Ubuntu computers;
        * Arduino (ATmega328P) microcontroler;
        * LPC21XX microcontroler.
    
    Currently supported link layers :
        * I²C (for arduino and LPC21XX)
        * UART (for arduino and linux)
        * Xbee (for arduino and linux)

Technical details :
    It relies on other API to handle the link layer, Xbee and UART especialy. See their 
   	documentation for details.
   	
   	Messages send are directly send (functions bn_send*** return after the actual sending or 
   	failure).
   	Incoming and in transit messages are put in a circular buffer, which size is determined by 
   	BN_INC_MSG_BUF_SIZE. The bigger this buffer is, the more burst traffic it can handle, but 
   	the higher the RAM use is. This buffer is emptied by calls to bn_receive or bn_routine. It is filled 
   	by these functions, but also on interruption (especially for Arduino).
   	
   	The configuration of the network is done with several files :
   	    * Global files : MUST be shared by every node of the network : 
            * messages.h : defines the different types of payloads.
            * network_cfg.c : defines the routing tables.
            * network_cfg.h : defines the addresses.
        * Local files : MUST be local per project and customized per node :
            * node_cfg.h : cf below.
    You can find an examples of fully working network in the testNet folder, with several types of nodes
    configured, and with proper global configuration.    
    
    
    
Requirements :
    Requires to define the following symbols :
	    ARCH_XXX  (currently supported XXX : 328P_ARDUINO & X86_LINUX & LPC21XX)
	    ARCH_BIG_ENDIAN xor ARCH_LITTLE_ENDIAN 
	
	Requires visibility on the global config files.
	
    Requires a visibility on a "node_cfg.h" file defining :

        #define MYADDRX xxx     // Xbee address (set to 0 if the current node doesn't use Xbee)
        #define MYADDRI xxx     // I²C address (set to 0 if the current node doesn't use I²C)
        #define MYADDRU xxx     // UART address (set to 0 if the current node doesn't use I²C)
        
        #define BN_INC_MSG_BUF_SIZE     xxx     // Size (in messages) of the incoming message buffer
                                                // RAM usage : xxx*68 bytes 
        #define BN_WAIT_XBEE_SND_FAIL   xxx     // Time before considering an Xbee sending 
                                                // as failed
        #define BN_MAX_RETRIES          xxx     // Number of call to level 2 send function 
                                                // before considering that sending has 
                                                // failed
        #define BN_ACK_TIMEOUT          xxx     // Acknowledgement timeout, in ms
        
        #define BN_UART_PATH "/dev/ttyACM0"

    Requires other libraries to be located at the same place as in the ENAC robotic club git 
    repository.

    Requires that Timeout library is correctly build and linked.
    
    Requires any library used to be build and linked. See their own documentation for details.
    List of (potentially) used libraries :
        * Wire (for arduino's I²C);
        * UART_framing;
        * Xbee_API.

Usage : 
    bn_init() must be called prior to any use of the bn API.
    bn_routine or bn_receive must be called regularly and rapidly (within the main loop 
    for example).
    The sending of a message must be done with bn_send or bn_sendAck. Prior to the call to 
    these functions, the following fields of the header of the message must be filled :
        *  Destination address (destAddr);
        *  Size of the payload (size);
        *  Type of the message (type).
    
    Refer to the comments in the source code for more details.

