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
#include <processes/ProcAbsPos.h>
#include <processes/ProcAbsPosSA.h>
#include <Plane3D.h>
#include <tools/AbsPos2D.h>
#include <tools/Acq.h>
#include <tools/Cam.h>
#include <tools/Image.h>
#include <tools/simulated_annealing.h>
#include <tools/Uncertainty2D.h>
#include <Transform2D.h>
#include <Vector2D.h>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

void ProcAbsPosSA::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
    assert(acqList.size() == 1);

    Acq* acq = acqList.front();

    Mat rgb = acq->getMat(BGR);

    Perf& perf = Perf::getPerf();

    static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
    ProjAcq pAcq = acq->projectOnPlane(pl);
    handleStart(pAcq, pos);

    cout << "  begpos: " << pos.x() << ", " << pos.y() << ", " << pos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, pos) << endl;


    const Vector2D<float> camDir_rob(pAcq.cam2plane(acq->getCam()->getCenter()));
    const float cm2rad = 1.f/camDir_rob.norm();

    ofstream fout_trials("out_trials.csv");
    fout_trials << "x,y,theta,E" << endl;

    AbsPos2D<float> endPos = simulated_annealing<AbsPos2D<float>, int, float>(pos, 20.f, 0.95f, 150, 12,
            [this, &pAcq, &fout_trials](AbsPos2D<float> const& pt) { // get energy
                float ret = this->getEnergy(pAcq, pt);

                fout_trials << pt.x() << "," << pt.y() << "," << pt.theta() << "," << ret << endl;

                return ret;
            },
            [this, &camDir_rob, &cm2rad, &pos, &posU](AbsPos2D<float> const& curr, int rem_c) { // get neighbor (4 terms: deltaX, deltaY, deltaTheta@Robot, deltaTheta@Image)
//                float per = 1.f - rem_c/400.f;

                constexpr float dt = 3.f; // (cm)
                float prop = 0;//0.25f + per/3.f; // (%)
                float dr = dt * (1.f - prop); // (cm)
                float di = dt * prop; // (cm)

                // compute dx/dy of robot for a corresponding rotation of image
                float dti = di * cm2rad * this->getRand();
                Vector2D<float> d_rob(camDir_rob - camDir_rob.rotated(dti));
                Vector2D<float> d_pg(d_rob.rotated(curr.theta()));

                float nx = clamp(curr.x() + dr * this->getRand() + d_pg.x, 20.f, 280.f);
                float ny = clamp(curr.y() + dr * this->getRand() + d_pg.y, 20.f, 180.f);
                float nt = curr.theta() + dr * cm2rad * this->getRand() + dti;

//                while(nt - pos.theta() > M_PI)
//                    nt -= 2.f*M_PI;
//                while(nt - pos.theta() < -M_PI)
//                    nt += 2.f*M_PI;
//
//                nt = clamp(nt, pos.theta() - posU.theta, pos.theta() + posU.theta);

                return AbsPos2D<float>(nx, ny, nt);
            });

    fout_trials.close();

    cout << "  endpos: " << endPos.x() << ", " << endPos.y() << ", " << endPos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, endPos) << endl;

    perf.endOfStep("ProcAbsPos::optim (simulated annealing)");

    handleEnd(pAcq, endPos);
}
