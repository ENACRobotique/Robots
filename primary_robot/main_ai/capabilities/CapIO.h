/*
 * CapIOProcess.h
 *
 *  Created on: 12 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPIO_H_
#define CAPABILITIES_CAPIO_H_

#include <Capability.h>

class CapIO : public Capability {
    public:
        CapIO(Robot* robot_init) : Capability(robot_init){};
        virtual ~CapIO(){};

// TODO setServo, getServo, getHMI, setHMI

        virtual unsigned int getHMI(const eIhmElement& el){
            return ihm.getValue(el);
        }

        virtual void setHMI(const eIhmElement& el, const unsigned int& value){
            ihm.sendIhm(el,value);
        }

        virtual void selectColor() {
            static int state = 0;

            switch(state) {
                case 0 :
                    if(getHMI(IHM_MODE_SWICTH) == 0) {
                        state = 1;
                        robot->color = YELLOW;
                        setHMI(IHM_LED, LED_YELLOW);
                    }
                    break;
                case 1 :
                    if(getHMI(IHM_MODE_SWICTH) == 1)
                        state = 2;
                    break;
                case 2 :
                    if(getHMI(IHM_MODE_SWICTH) == 0) {
                        state = 3;
                        robot->color = GREEN;
                        setHMI(IHM_LED, LED_GREEN);
                    }
                    break;
                case 3 :
                if(mode_switch == 1)
                    state = 0;
                break;
            }
        }
};


#endif /* CAPABILITIES_CAPIO_H_ */
