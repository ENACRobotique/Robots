/*
 * ProcAbsPosSA.h
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#ifndef PROCESSES_PROCABSPOSSA_H_
#define PROCESSES_PROCABSPOSSA_H_

#include <processes/ProcAbsPos.h>

class ProcAbsPosSA: public ProcAbsPos {
public:
    ProcAbsPosSA(Cam* c, const std::string& staticTestPointFile) : ProcAbsPos(c, staticTestPointFile) {}
    virtual ~ProcAbsPosSA(){}

    void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) override;
};

#endif /* PROCESSES_PROCABSPOSSA_H_ */
