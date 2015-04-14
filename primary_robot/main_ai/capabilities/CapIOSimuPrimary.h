/*
 * CapIOSimuPrimary.h
 *
 *  Created on: 14 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPIOSIMUPRIMARY_H_
#define CAPABILITIES_CAPIOSIMUPRIMARY_H_

#include <CapIO.h>

class CapIOSimuPrimary : public CapIO {
    public:
        CapIOSimuPrimary(Robot* robot_init) : CapIO(robot_init){};
        ~CapIOSimuPrimary(){};

        unsigned int getValue(const eIhmElement& el){
            static eIhmCord cord = CORD_OUT;

            switch(el){
                case IHM_STARTING_CORD:
                    if(cord == CORD_OUT)
                        cord = CORD_IN;
                    else
                        cord = CORD_OUT;
                    return static_cast <unsigned int>(cord);
                case IHM_MODE_SWICTH:
                    if(robot->color == YELLOW)
                        return static_cast <unsigned int>(eIhmSwitch::SWITCH_ON); //FIXME link with the real
                    else
                        return static_cast <unsigned int>(eIhmSwitch::SWITCH_OFF); //FIXME link with the real
                case IHM_LED:
                    if(robot->color == YELLOW)
                        return static_cast <unsigned int>(eIhmLed::LED_YELLOW);
                    else if(robot->color == GREEN)
                        return static_cast <unsigned int>(eIhmLed::LED_GREEN);
                    else
                        return static_cast <unsigned int>(eIhmLed::LED_OFF);
                default:
                    return 0;
            }
        }
};


#endif /* CAPABILITIES_CAPIOSIMUPRIMARY_H_ */
