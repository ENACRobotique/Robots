/*
 * CapPrepPrimary.cpp
 *
 *  Created on: 18 avr. 2015
 *      Author: Sébastien Malissard
 */

#include "CapPrepPrimary.h"
#include "CapIO.h"
#include "CapTeam.h"
#include "CapAI.h"
#include "tools.h"
#include "communications.h"
extern "C"{
#include "millis.h"
}


int CapPrepPrimary::loop(){
    CapIO* capIO = dynamic_cast<CapIO*> (robot->caps[eCap::IO]);
    CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);

    if(!capIO){
        logs << ERR << "Primary must be have an HMI interface real or simulate";
        exit(EXIT_FAILURE);
    }

    switch (_step){
    case Step::COLOR_SELECTION: //Choose the color and take off the starting cord
        capIO->selectColor();

        if(capIO->getHMI(IHM_STARTING_CORD) == CORD_OUT) {
            float theta_robot;
            Point2D<float> pos_robot;

            if (capTeam->getColor() == YELLOW) {
                logs << INFO << "Color selected is YELLOW";
                pos_robot = {INIT_POS_YELLOW_X, INIT_POS_YELLOW_Y};
                theta_robot = INIT_ANGLE_YELLOW;
            }
            else if (capTeam->getColor() == GREEN) {
                logs << INFO << "Color selected is GREEN";
                pos_robot = {INIT_POS_GREEN_X, INIT_POS_GREEN_Y};
                theta_robot = INIT_ANGLE_GREEN;
            }
            else {
                logs << ERR << "Error selection color";
                return -1;
            }

            sendPosPrimary(pos_robot, theta_robot);            // Sending approximate initial position to the propulsion

            //TODO procedure de mise en place

            logs << INFO << "End step color selection";
            _step = Step::WAIT_STARTING_CORD;
        }
        break;

    case Step::WAIT_STARTING_CORD: //Wait to take in the starting cord
        if(capIO->getHMI(IHM_STARTING_CORD) == CORD_IN){
            if(CapAI* capAI = dynamic_cast<CapAI*> (robot->caps[eCap::AI]))
                capAI->initObjective();

            logs << INFO << "End step wait starting cord";
            _step = Step::WAIT_START;
        }
        break;

    case Step::WAIT_START: //Wait the start (take off the starting cord)
        if (capIO->getHMI(IHM_STARTING_CORD) == CORD_OUT) {
            capTeam->setStartGame(millis());

            logs << INFO << "End step wait start";
            _step = Step::GAME;
        }
        break;

    case Step::GAME: //Let's go
        logs << INFO << "Start game : Let's go!!!!";
        return 1;
    }

    return 0;
}
