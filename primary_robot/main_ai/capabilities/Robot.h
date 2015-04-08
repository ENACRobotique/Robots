/*
 * Robot.h
 *
 *  Created on: 30 mars 2015
 *      Author: seb
 */

#ifndef CAPABILITIES_ROBOT_H_
#define CAPABILITIES_ROBOT_H_

#include <capabilities/Capability.h>
#include <map>
#include <vector>

#include "messages-elements.h"
#include "tools.h"

typedef enum {
    POS,
    AI,
    PROP,
    HOLO,
    AXLE
}eCap;


class Robot {
    public:
        Robot(std::string name, eElement el) : _el(el){
            switch(el){
                case(ELT_PRIMARY):
                    _name = "Primary";
                    break;
                case(ELT_SECONDARY):
                    _name = "Secondary";
                    break;
                case(ELT_ADV_PRIMARY):
                    _name = "AdvPrimary";
                    break;
                case(ELT_ADV_SECONDARY):
                    _name = "AdvSecondary";
                    break;
                default:
                    logs << ERR << "This element is not a robot";
            }
            if(!name.empty())
                _name << ":" << name;
        }
        ~Robot(){}


    public:
        std::string _name;
        eElement _el;
        std::map<eCap, Capability*> cap;
};

#endif /* CAPABILITIES_ROBOT_H_ */
