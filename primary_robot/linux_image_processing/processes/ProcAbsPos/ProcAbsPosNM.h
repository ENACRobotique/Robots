/*
 * ProcAbsPosNM.h
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#ifndef PROCESSES_PROCABSPOSNM_H_
#define PROCESSES_PROCABSPOSNM_H_

#include "ProcAbsPos.h"

class ProcAbsPosNM: public ProcAbsPos {
public:
    ProcAbsPosNM(Cam* c, const std::string& staticTestPointFile) : ProcAbsPos(c, staticTestPointFile) {}
    virtual ~ProcAbsPosNM(){}

    void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) override;
};

#endif /* PROCESSES_PROCABSPOSNM_H_ */
