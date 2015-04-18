/*
 * ProcAbsPos.cpp
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#include <processes/ProcAbsPos.h>
#include <Point2D.h>

using namespace std;

ProcAbsPos::ProcAbsPos(const string& staticTestPointFile)
        {
    // TODO read input file and fill staticTP vector

    staticTP.push_back(TestPoint(Pt(10, 10), 0., 1.));
}

ProcAbsPos::~ProcAbsPos()
{
}

void ProcAbsPos::process(const std::vector<Acq*>& acqList) {
    // TODO
}
