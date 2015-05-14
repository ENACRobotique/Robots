/*
 * CapPrepPrimary.h
 *
 *  Created on: 18 avr. 2015
 *      Author: seb
 */

#ifndef CAPABILITIES_CAPPREPPRIMARY_H_
#define CAPABILITIES_CAPPREPPRIMARY_H_

#include "CapPreparation.h"

#define INIT_POS_YELLOW_X 45
#define INIT_POS_YELLOW_Y 100
#define INIT_POS_GREEN_X  (300 - INIT_POS_YELLOW_X)
#define INIT_POS_GREEN_Y  INIT_POS_YELLOW_Y
#define INIT_ANGLE_YELLOW 0
#define INIT_ANGLE_GREEN  M_PI

typedef enum {
    COLOR_SELECTION, WAIT_STARTING_CORD, WAIT_START, GAME
} Step;

class CapPrepPrimary: public CapPreparation {
    public:
        CapPrepPrimary(Robot* robot_init) :
            CapPreparation(robot_init), _step(COLOR_SELECTION) {
        }
        ~CapPrepPrimary() {
        }

        int loop();

    private:
       Step  _step;

};





#endif /* CAPABILITIES_CAPPREPPRIMARY_H_ */
