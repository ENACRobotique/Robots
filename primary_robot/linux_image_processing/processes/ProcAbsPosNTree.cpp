/*
 * ProcAbsPosSA.cpp
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#include <performance.hpp>
#include <processes/ProcAbsPosNTree.h>
#include <Plane3D.h>
#include <tools/AbsPos2D.h>
#include <tools/Acq.h>
#include <tools/Cam.h>
#include <tools/Image.h>
#include <tools/n_tree.h>
#include <tools/RelPos2D.h>
#include <tools/Uncertainty2D.h>
#include <Vector2D.h>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

void ProcAbsPosNTree::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
    assert(acqList.size() == 1);

    Acq* acq = acqList.front();

    Mat rgb = acq->getMat(BGR);

    Perf& perf = Perf::getPerf();

    static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
    ProjAcq pAcq = acq->projectOnPlane(pl);
    handleStart(pAcq, pos);

    cout << "  begpos: " << pos.x() << ", " << pos.y() << ", " << pos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, pos) << endl;

    const Vector2D<float> camDir_rob(pAcq.cam2plane(acq->getCam()->getCenter()));

    ofstream fout_trials("out_trials.csv");
    fout_trials << "x,y,theta,E" << endl;

    const RelPos2D<float> vecX(sqrt(posU.a_var), 0, 0);
    const RelPos2D<float> vecY(0, sqrt(posU.b_var), 0);
    const RelPos2D<float> vecT(0, 0, posU.theta);

    AbsPos2D<float> endPos = n_tree<float, AbsPos2D<float>, 3>(pos, 0.f, 6, 2,
            [this, &pAcq, &fout_trials](AbsPos2D<float> const& pt) {
                float ret = this->getEnergy(pAcq, pt);

                fout_trials << pt.x() << "," << pt.y() << "," << pt.theta() << "," << ret << endl;

                return ret;
            },
            [&vecX, &vecY, &vecT](AbsPos2D<float> const& pt, int iter) {
                float fact = pow(2, -(iter + 1));

                return std::array<AbsPos2D<float>, 8>{
                    pt + vecX * fact + vecY * fact - vecT * fact,
                    pt + vecX * fact + vecY * fact + vecT * fact,
                    pt + vecX * fact - vecY * fact - vecT * fact,
                    pt + vecX * fact - vecY * fact + vecT * fact,
                    pt - vecX * fact + vecY * fact - vecT * fact,
                    pt - vecX * fact + vecY * fact + vecT * fact,
                    pt - vecX * fact - vecY * fact - vecT * fact,
                    pt - vecX * fact - vecY * fact + vecT * fact
                };
            });

//    AbsPos2D<float> endPos = simulated_annealing<AbsPos2D<float>, int, float>(pos, 20.f, 1.f, 400, 400,
////    AbsPos2D<float> endPos = simulated_annealing<AbsPos2D<float>, int, float>(pos, 20.f, 0.9626f, 150, 12,
//            [this, &pAcq, &fout_trials](AbsPos2D<float> const& pt) { // get energy
//                float ret = this->getEnergy(pAcq, pt);
//
//                fout_trials << pt.x() << "," << pt.y() << "," << pt.theta() << "," << ret << endl;
//
//                return ret;
//            },
//            [this, &camDir_rob, &cm2rad, &pos, &posU](AbsPos2D<float> const& curr, int rem_c) { // get neighbor (4 terms: deltaX, deltaY, deltaTheta@Robot, deltaTheta@Image)
////                float per = 1.f - rem_c/400.f;
//
//                constexpr float dt = 3.f; // (cm)
//                float prop = 0;//0.25f + per/3.f; // (%)
//                float dr = dt * (1.f - prop); // (cm)
//                float di = dt * prop; // (cm)
//
//                // compute dx/dy of robot for a corresponding rotation of image
//                float dti = di * cm2rad * this->getRand();
//                Vector2D<float> d_rob(camDir_rob - camDir_rob.rotated(dti));
//                Vector2D<float> d_pg(d_rob.rotated(curr.theta()));
//
//                float nx = clamp(curr.x() + dr * this->getRand() + d_pg.x, pos.x() - sqrt(posU.a_var), pos.x() + sqrt(posU.a_var));
//                float ny = clamp(curr.y() + dr * this->getRand() + d_pg.y, pos.y() - sqrt(posU.b_var), pos.y() + sqrt(posU.b_var));
//                float nt = curr.theta() + dr * cm2rad * this->getRand() + dti;
//
//                while(nt - pos.theta() > M_PI)
//                    nt -= 2.f*M_PI;
//                while(nt - pos.theta() < -M_PI)
//                    nt += 2.f*M_PI;
//
//                nt = clamp(nt, pos.theta() - posU.theta, pos.theta() + posU.theta);
//
//                return AbsPos2D<float>(nx, ny, nt);
//            });

    fout_trials.close();

    cout << "  endpos: " << endPos.x() << ", " << endPos.y() << ", " << endPos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, endPos) << endl;

    perf.endOfStep("ProcAbsPos::optim (simulated annealing)");

    handleEnd(pAcq, endPos);
}
