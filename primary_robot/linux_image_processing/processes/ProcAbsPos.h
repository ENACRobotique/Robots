/*
 * ProcAbsPos.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef PROCESSES_PROCABSPOS_H_
#define PROCESSES_PROCABSPOS_H_

#include <processes/Process.h>
#include <tools/Acq.h>
#include <tools/TestPoint.h>
#include <string>
#include <vector>

class ProcAbsPos: public Process {
protected:
    std::vector<TestPoint> staticTP;

    float getEnergy(Acq& acq, Pos); // WIP

public:
    ProcAbsPos(const std::string& staticTestPointFile);
    virtual ~ProcAbsPos();

    virtual void process(const std::vector<Acq*>& acqList) override;
};

#endif /* PROCESSES_PROCABSPOS_H_ */
