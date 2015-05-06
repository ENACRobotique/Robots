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
    uint16_t nb_servos; // must be <=10
    struct __attribute__((packed)){ // 5 bytes
    	uint16_t id; // identifier of the servomotor (club number)
    	uint16_t pca_id; // plug number (on the card 0-15)
        float angle; // servo setpoint in degrees (0-*)
    } servos[];
} sServos;

typedef enum{
    IHM_STARTING_CORD,
    IHM_MODE_SWITCH,
    IHM_LED,
    IHM_LIMIT_SWITCH_RIGHT,
    IHM_LIMIT_SWITCH_LEFT
} eIhmElement;

typedef enum {
    CORD_IN,
    CORD_OUT
} eIhmCord;

typedef enum {
    SWITCH_ON,
    SWITCH_OFF,
    SWITCH_INTER
} eIhmSwitch;

typedef enum {
    LED_OFF,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
	LED_YELLOW,
	LED_MAGENTA,
	LED_CYAN,
	LED_WHITE,
	LED_RED_BLINK,
	LED_GREEN_BLINK,
	LED_BLUE_BLINK,
	LED_YELLOW_BLINK,
	LED_PURPLE_BLINK,
	LED_CYAN_BLINK,
	LED_WHITE_BLINK
}eIhmLed;

typedef struct __attribute__((packed)){
    uint16_t nb_states; // must be <=18
    struct __attribute__((packed)){
        eIhmElement id :8; // identifier of the ihm element
        union{
            uint16_t state; // generic status
            eIhmCord state_cord :8;
            eIhmSwitch state_switch :8;
            eIhmLed sate_led :8;
        };
    } states[];
} sIhmStatus;


#endif /* LIB_NETWORK_CONFIG_MESSAGES_INTERACTIONS_H_ */
