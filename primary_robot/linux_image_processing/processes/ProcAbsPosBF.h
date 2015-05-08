/*
 * ProcAbsPosSA.h
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#ifndef PROCESSES_PROCABSPOSBF_H_
#define PROCESSES_PROCABSPOSBF_H_

#include <processes/ProcAbsPos.h>

class ProcAbsPosBF: public ProcAbsPos {
public:
    ProcAbsPosBF(Cam* c, const std::string& staticTestPointFile) : ProcAbsPos(c, staticTestPointFile) {}
    virtual ~ProcAbsPosBF(){}

    void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) override;
};

#endif /* PROCESSES_PROCABSPOSBF_H_ */
