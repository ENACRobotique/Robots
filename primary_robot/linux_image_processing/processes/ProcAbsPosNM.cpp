/*
 * ProcAbsPosSA.cpp
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <performance.hpp>
#include <processes/ProcAbsPosNM.h>
#include <Plane3D.h>
#include <tools/AbsPos2D.h>
#include <tools/Acq.h>
#include <tools/Cam.h>
#include <tools/Image.h>
#include <tools/neldermead.h>
#include <Transform2D.h>
#include <Vector2D.h>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

void ProcAbsPosNM::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
    assert(acqList.size() == 1);

    Acq* acq = acqList.front();

    Mat rgb = acq->getMat(BGR);

    Perf& perf = Perf::getPerf();

    static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
    ProjAcq pAcq = acq->projectOnPlane(pl);
    handleStart(pAcq, pos);

    cout << "  begpos: " << pos.x() << ", " << pos.y() << ", " << pos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, pos) << endl;

    float du = 10;
    float dv = 10;
    float cm2rad = 1.f / 40.f;

    Transform2D<float> tr_pg2rob = pos.getTransform();
    Transform2D<float> tr_rob2pg = tr_rob2pg.getReverse();

    for (int i = 0; i < 1; i++) {
        float ct = cos(pos.theta());
        float st = sin(pos.theta());

        array<AbsPos2D<float>, 4> simplex {
            AbsPos2D<float>(
                    pos.x() - ct * du,
                    pos.y() - st * du,
                    pos.theta() + du * cm2rad),
            AbsPos2D<float>(
                    pos.x() - st * dv,
                    pos.y() + ct * dv,
                    pos.theta()),
            AbsPos2D<float>(
                    pos.x() + ct * du,
                    pos.y() + st * du,
                    pos.theta() + du * cm2rad),
            AbsPos2D<float>(
                    pos.x() + st * dv,
                    pos.y() - ct * dv,
                    pos.theta()),
        };

        cout << "input simplex:" << endl;
        for (AbsPos2D<float> const& v : simplex) {
            cout << v.x() << " " << v.y() << " " << v.theta() << endl;
        }

        perf.endOfStep("ProcAbsPosNM::prepare optim");

        AbsPos2D<float> endPos = neldermead<float, AbsPos2D<float>, 3>(simplex, 0, 30,
                [this, &pAcq](AbsPos2D<float> const& pt, int iter) {
                    float enr = this->getEnergy(pAcq, pt);

                    // cout << "f(pt = " << pt << ", iter = " << iter << ") = " << enr << " + " << rnd / enr * 100.f << " %" << endl;

                    return enr;
                });

        cout << "output simplex:" << endl;
        for (AbsPos2D<float> const& v : simplex) {
            cout << v.x() << " " << v.y() << " " << v.theta() << endl;
        }

        cout << "  endpos: " << endPos.x() << ", " << endPos.y() << ", " << endPos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, endPos) << endl;

        perf.endOfStep("ProcAbsPosNM::optim #" + to_string(i));

        handleStep(pAcq, endPos, i);
    }

}
