/*
 * CapIOSimuPrimary.h
 *
 *  Created on: 14 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPIOSIMUPRIMARY_H_
#define CAPABILITIES_CAPIOSIMUPRIMARY_H_

#include <CapIO.h>
#include "CapTeam.h"

#include "tools.h"

class CapIOSimuPrimary : public CapIO {
    public:
        CapIOSimuPrimary(Robot* robot_init) : CapIO(robot_init){};
        ~CapIOSimuPrimary(){};

        unsigned int getHMI(const eIhmElement& el){
            static eIhmCord cord = CORD_OUT;
            CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);

            switch(el){
                case IHM_STARTING_CORD:
                    if(cord == CORD_OUT)
                        cord = CORD_IN;
                    else
                        cord = CORD_OUT;
                    return static_cast <unsigned int>(cord);
                case IHM_MODE_SWITCH:
                    return static_cast <unsigned int>(eIhmSwitch::SWITCH_OFF);
                case IHM_LED:
                    if(capTeam->getColor() == eColor_t::YELLOW)
                        return static_cast <unsigned int>(eIhmLed::LED_YELLOW);
                    else if(capTeam->getColor() == eColor_t::GREEN)
                        return static_cast <unsigned int>(eIhmLed::LED_GREEN);
                    else
                        return static_cast <unsigned int>(eIhmLed::LED_OFF);
                default:
                    return 0;
            }
        }

        void selectColor(){}
};


#endif /* CAPABILITIES_CAPIOSIMUPRIMARY_H_ */
