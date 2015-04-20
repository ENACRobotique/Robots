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
        CapTeam(Robot* robot_init, eColor_t color) : Capability(robot_init), _color(color), _start_game(0){}
        ~CapTeam(){}


        void setColor(eColor_t color){
            _color = color;
        }

        void setStartGame(unsigned int start_game){
            _start_game = start_game;
        }

        eColor_t getColor(){
            return _color;
        }

        vector<objective> getObjective(){
            return _el;
        }

        unsigned int getStartGame(){
            return _start_game;
        }

    private:
        eColor_t _color;
        vector<objective> _el;
        unsigned int _start_game;
};




#endif /* CAPABILITIES_CAPTEAM_H_ */
