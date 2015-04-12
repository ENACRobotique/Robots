/*
 * CapIOProcIHM.h
 *
 *  Created on: 12 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPIOPROCHMI_H_
#define CAPABILITIES_CAPIOPROCHMI_H_

#include <vector>

#include <Capability.h>
#include "CapIOProcess.h"
#include "messages-interactions.h"
#include "tools.h"

class CapIOProcHMI : public CapIOProcess {
    public:
        CapIOProcHMI(Robot* robot_init, std::vector<eIhmElement>& el) : CapIOProcess(robot_init), _el(el){};
        ~CapIOProcHMI(){};

        int getValue(const eIhmElement& el){
            //TODO get to ihm class
            return el;
        }

    private:
        std::vector<eIhmElement> _el;

};



#endif /* CAPABILITIES_CAPIOPROCHMI_H_ */
