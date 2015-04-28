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
    float dr = 0;
    float cm2rad = 1.f / 40.f;

    Transform2D<float> tr_pg2rob = pos.getTransform();
    Transform2D<float> tr_rob2pg = tr_rob2pg.getReverse();
    Vector2D<float> camDir(pAcq.cam2plane(acq->getCam()->getCenter()));

    cout << "camDir: " << camDir << endl;

    for (int i = 0; i < 1; i++) {
        float t = pos.theta() + dr * cm2rad * getRand();
        float ct = cos(t);
        float st = sin(t);

        array<AbsPos2D<float>, 4> simplex {
            AbsPos2D<float>(
                    pos.x() - ct * du +
                    dr * getRand(),
                    pos.y() - st * du +
                    dr * getRand(),
                    pos.theta() - du * cm2rad +
                    dr * cm2rad * getRand(),
                    camDir),
            AbsPos2D<float>(
                    pos.x() - st * dv +
                    dr * getRand(),
                    pos.y() + ct * dv +
                    dr * getRand(),
                    pos.theta() +
                    dr * cm2rad * getRand(),
                    camDir),
            AbsPos2D<float>(
                    pos.x() + ct * du +
                    dr * getRand(),
                    pos.y() + st * du +
                    dr * getRand(),
                    pos.theta() + du * cm2rad +
                    dr * cm2rad * getRand(),
                    camDir),
            AbsPos2D<float>(
                    pos.x() + st * dv +
                    dr * getRand(),
                    pos.y() - ct * dv +
                    dr * getRand(),
                    pos.theta() - du * cm2rad +
                    dr * cm2rad * getRand(),
                    camDir),
        };

        cout << "input simplex:" << endl;
        for (AbsPos2D<float> const& v : simplex) {
            cout << v.x() << " " << v.y() << " " << v.theta() << endl;
        }

        perf.endOfStep("ProcAbsPos::setting up optim");

        AbsPos2D<float> endPos = neldermead<float, AbsPos2D<float>, 3>(simplex, [this, &pAcq](AbsPos2D<float> const& pt, int iter){
            float enr = this->getEnergy(pAcq, pt);
            float rnd = iter >= 0 ? enr * 1.f * this->getRand() / log(3.f + iter) : 0.f;
            float ret = enr + rnd;

        // cout << "f(pt = " << pt << ", iter = " << iter << ") = " << enr << " + " << rnd / enr * 100.f << " %" << endl;

            return ret;
        }, 0, 30);


        cout << "output simplex:" << endl;
        for (AbsPos2D<float> const& v : simplex) {
            cout << v.x() << " " << v.y() << " " << v.theta() << endl;
        }

        cout << "  endpos: " << endPos.x() << ", " << endPos.y() << ", " << endPos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, endPos) << endl;

        perf.endOfStep("ProcAbsPos::optim (neldermead #" + to_string(i) + ")");


        handleStep(pAcq, endPos, i);
    }

}
