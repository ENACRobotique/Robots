/*
 * ProcAbsPosNTree2.h
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#ifndef PROCESSES_PROCABSPOSNTREE2_H_
#define PROCESSES_PROCABSPOSNTREE2_H_

#include "ProcAbsPos.h"

class ProcAbsPosNTree2: public ProcAbsPos {
public:
    ProcAbsPosNTree2(Cam* c, const std::string& staticTestPointFile, eVidTypeProc typeProcess) : ProcAbsPos(c, staticTestPointFile, typeProcess) {}
    virtual ~ProcAbsPosNTree2(){}

    void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) override;
};

#endif /* PROCESSES_PROCABSPOSNTREE2_H_ */
