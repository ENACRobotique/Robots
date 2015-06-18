/*
 * ihm.h
 *
 *  Created on: 23 févr. 2015
 *      Author: seb
 */

#ifndef MAIN_AI_TOOLS_IHM_H_
#define MAIN_AI_TOOLS_IHM_H_

#include <map>

#include "messages-interactions.h"
#include "messages-elements.h"

#define MAX_IHM_ELEMENTS 10

using namespace std;

typedef enum{
    LED_OFF,
    LED_YELLOW,
    LED_GREEN
}eIhmLed;

eIhmLed ihmLedConvertRGBToEnum(sRGB rgb);
sRGB ihmLedConvertEnumToRgb(eIhmLed led);

class Ihm {
    public:
        Ihm(){}
        ~Ihm(){}

        void receivedNewIhm(sIhmStatus& ihm);
        void sendIhm(const eIhmElement& id, const unsigned int& state);

        eIhmCord getStartingCord(){
            return static_cast <eIhmCord>(list[eIhmElement::IHM_STARTING_CORD]);
        }
        eIhmSwitch getSwitchColor(){
            return static_cast <eIhmSwitch>(list[eIhmElement::IHM_MODE_SWITCH]);
        }
        eIhmLed getLed(){
            return static_cast <eIhmLed>(list[eIhmElement::IHM_LED]);
        }
        unsigned int getValue(const eIhmElement& id){
            return list[id];
        }

    private:
        map <eIhmElement, unsigned int> list;
};

#endif /* MAIN_AI_TOOLS_IHM_H_ */
