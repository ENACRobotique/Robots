/*
 * ihm.h
 *
 *  Created on: 23 févr. 2015
 *      Author: seb
 */

#ifndef MAIN_AI_TOOLS_IHM_H_
#define MAIN_AI_TOOLS_IHM_H_

#include "messages-interactions.h"

using namespace std;

typedef enum {
    IN, OUT, UNKNOWN_INOUT
} eInOut_t;

typedef enum {
    ON, OFF, UNKNOWN_SWITCH
} eSwitch_t;

typedef enum {
    LED_OFF, LED_RED, LED_YELLOW, UNKNOWN_LED
}eLed_t;

class Ihm {
    public:
        Ihm();
        ~Ihm();

        void receivedNewIhm(sIhmStatus &ihm);

        eInOut_t getStartingCord();
        eSwitch_t getSwitchColr();
        eLed_t getLed();

    private:
        eInOut_t _starting_cord;
        eSwitch_t _switch_color;
        eLed_t _led;
};

#endif /* MAIN_AI_TOOLS_IHM_H_ */
