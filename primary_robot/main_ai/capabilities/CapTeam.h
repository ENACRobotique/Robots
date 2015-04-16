/*
 * CapTeam.h
 *
 *  Created on: 16 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPTEAM_H_
#define CAPABILITIES_CAPTEAM_H_

#include <Capability.h>

#include <vector>

typedef enum{
    SPOT,
    CLAP,
    POP_CORN
}objective;

class CapTeam : public Capability {
    public:
        CapTeam(Robot* robot_init, eColor_t color) : Capability(robot_init), _color(color){}
        ~CapTeam(){}


        void setColor(eColor_t color){
            _color = color;
        }

        eColor_t getColor(){
            return _color;
        }

        vector<objective> getObjective(){
            return _el;
        }

    private:
        eColor_t _color;
        vector<objective> _el;
};




#endif /* CAPABILITIES_CAPTEAM_H_ */
