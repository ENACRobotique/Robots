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

class Ihm {
    public:
        Ihm();
        ~Ihm();

        void receivedNewIhm(sIhmStatus &ihm);

        eIhmCord getStartingCord();
        eIhmSwitch getSwitchColr();
        eIhmLed getLed();

    private:
        eIhmCord _starting_cord;
        eIhmSwitch _switch_color;
        eIhmLed _led;
};

#endif /* MAIN_AI_TOOLS_IHM_H_ */
