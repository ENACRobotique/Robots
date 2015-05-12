/*
 * messages-interactions.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_INTERACTIONS_H_
#define LIB_NETWORK_CONFIG_MESSAGES_INTERACTIONS_H_

#include <stdint.h>


typedef struct __attribute__((packed)){
    uint16_t nb_servos; // must be <=9
    struct __attribute__((packed)){ // 5 bytes
    	uint8_t club_id; // identifier of the servomotor (club number)
    	uint8_t hw_id; // plug number (on the card 0-15)
        float angle; // servo setpoint in degrees (0-*)
    } servos[];
} sServos;

typedef enum{
    IHM_STARTING_CORD,
    IHM_MODE_SWITCH,
    IHM_LED,
    IHM_PRESENCE_1,
	IHM_PRESENCE_2,
	IHM_PRESENCE_3,
	IHM_PRESENCE_4,
	IHM_PRESENCE_5
} eIhmElement;

typedef enum {
    CORD_OUT,
    CORD_IN
} eIhmCord;

typedef enum {
    SWITCH_ON,
    SWITCH_OFF,
    SWITCH_INTER
} eIhmSwitch;

typedef enum {
	PRESENCE_FALSE,
	PRESENCE_TRUE
} eIhmPresence;


typedef struct __attribute__((packed)){
	uint8_t red, green, blue;
} sRGB;


typedef struct __attribute__((packed)){
    uint16_t nb_states; // must be <=4
    struct __attribute__((packed)){
        eIhmElement id :8; // identifier of the ihm element
        union __attribute__((packed)){
        	eIhmCord state_cord :8;
        	eIhmSwitch state_switch :8;
        	eIhmPresence state_presence :8;
        	struct __attribute__((packed)){
        		sRGB color1; //first color
        		sRGB color2; //second color
        		uint16_t time1 :14; //time while 1st color is on (ms)
        		uint16_t time2 :14; //time while 2nd color is on (ms)
        		uint8_t nb :4;  //number of periods (0 for infinite loop)
        	};
        } state;
    } states[];
} sIhmStatus;


#endif /* LIB_NETWORK_CONFIG_MESSAGES_INTERACTIONS_H_ */
