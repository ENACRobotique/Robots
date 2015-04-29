/*
 * ProcAbsPosSA.h
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#ifndef PROCESSES_PROCABSPOSNTREE_H_
#define PROCESSES_PROCABSPOSNTREE_H_

#include <processes/ProcAbsPos.h>

class ProcAbsPosNTree: public ProcAbsPos {
public:
    ProcAbsPosNTree(Cam* c, const std::string& staticTestPointFile) : ProcAbsPos(c, staticTestPointFile) {}
    virtual ~ProcAbsPosNTree(){}

    void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) override;
};

#endif /* PROCESSES_PROCABSPOSNTREE_H_ */
