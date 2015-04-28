/*
 * ProcAbsPosSA.h
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#ifndef PROCESSES_PROCABSPOSFULL_H_
#define PROCESSES_PROCABSPOSFULL_H_

#include <processes/ProcAbsPos.h>

class ProcAbsPosFull: public ProcAbsPos {
public:
    ProcAbsPosFull(Cam* c, const std::string& staticTestPointFile) : ProcAbsPos(c, staticTestPointFile) {}
    virtual ~ProcAbsPosFull(){}

    void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) override;
};

#endif /* PROCESSES_PROCABSPOSFULL_H_ */
