/*
 * Robot.h
 *
 *  Created on: 30 mars 2015
 *      Author: seb
 */

#ifndef CAPABILITIES_ROBOT_H_
#define CAPABILITIES_ROBOT_H_

#include <Environment.h>
#include <map>
#include <vector>

#include "messages-elements.h"
#include "tools.h"

typedef enum {
    POS,
    AI,
    SLAVE,
    PROP,
    BEACON, //balise mobile
    IO,
    VIDEO,
    TEAM,
    PREP,
    ACTUATOR
}eCap;

class Capability;

class Robot{
    public:
        Robot(std::string name_, eElement el_, Environment* env_) : env(env_), el(el_){
            switch(el){
                case(ELT_PRIMARY):
                    name = "Primary";
                    break;
                case(ELT_SECONDARY):
                    name = "Secondary";
                    break;
                case(ELT_ADV_PRIMARY):
                    name = "AdvPrimary";
                    break;
                case(ELT_ADV_SECONDARY):
                    name = "AdvSecondary";
                    break;
                default:
                    logs << ERR << "This element is not a robot";
            }
            if(!name.empty())
                name += ":" + name_;
        }
        ~Robot(){}

    public:
        Environment* env;
        std::string name;
        eElement el;
        std::map<eCap, Capability*> caps;
};

#endif /* CAPABILITIES_ROBOT_H_ */
