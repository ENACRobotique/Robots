/*
 * environment.h
 *
 *  Created on: 19 avr. 2015
 *      Author: Sebastien Malissard
 */

#ifndef CAPABILITIES_ENV_H_
#define CAPABILITIES_ENV_H_

#include <vector>

#include "a_star_tools.h"

class Environment{
    public:
        Environment(std::vector<astar::sObs_t>& obs_) {
            std::vector<uint8_t> obs_updatedy(obs_.size());
            obs = obs_;
            obs_updated = obs_updatedy;

        }
        ~Environment(){}


    public:
        std::vector<astar::sObs_t> obs;
        std::vector<uint8_t> obs_updated; //INIT
};


#endif /* CAPABILITIES_ENV_H_ */
