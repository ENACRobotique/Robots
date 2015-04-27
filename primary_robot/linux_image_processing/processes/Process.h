/*
 * Process.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef PROCESSES_PROCESS_H_
#define PROCESSES_PROCESS_H_

#include <tools/AbsPos2D.h>
#include <tools/Acq.h>
#include <tools/Cam.h>
#include <vector>

template<typename T> class Uncertainty2D;

class Acq;

template<typename T> class AbsPos2D;

class Process {
protected:
    std::vector<Cam*> camList;

public:
    using Pos = AbsPos2D<float>;
    using PosU = Uncertainty2D<float>;
    using Pt = Point2D<float>;

    virtual ~Process() {
    }

    std::vector<Cam*> getCamList() {
        return camList;
    }

    virtual void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) = 0;
};

#endif /* PROCESSES_PROCESS_H_ */
