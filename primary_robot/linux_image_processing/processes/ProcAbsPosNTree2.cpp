/*
 * ProcAbsPosNTree2.cpp
 *
 *  Created on: 29 avr. 2015
 *      Author: ludo6431
 */

#include <performance.hpp>
#include <processes/ProcAbsPosNTree2.h>
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

void ProcAbsPosNTree2::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
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

    perf.endOfStep("ProcAbsPosNTree2::prepare optim");

    AbsPos2D<float> currPos = pos;

    const RelPos2D<float> vecX(sqrt(posU.a_var), 0, 0);
    const RelPos2D<float> vecY(0, sqrt(posU.b_var), 0);
    const RelPos2D<float> vecT(0, 0, posU.theta);
    for (int iter = 0; iter < 2; iter++) {

        // x, y, theta   robot
        currPos = n_tree<float, AbsPos2D<float>, 3>(currPos, 0.f, 7 + 2*iter, 7 - iter,
                [this, &pAcq, &fout_trials](AbsPos2D<float> const& pt) {
                    float ret = this->getEnergy(pAcq, pt);

                    fout_trials << pt.x() << "," << pt.y() << "," << pt.theta() << "," << ret << endl;

                    return ret;
                },
                [&vecX, &vecY, &vecT](AbsPos2D<float> const& pt, int iter) {
                    const float fact = pow(2, -(iter + 1));

                    return std::array<AbsPos2D<float>, 8> {
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

        cout << "  endpos: " << currPos.x() << ", " << currPos.y() << ", " << currPos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, currPos) << endl;

//        // theta image
//        currPos = n_tree<float, AbsPos2D<float>, 1>(currPos, 0.f, 5, 2,
//                [this, &pAcq, &fout_trials](AbsPos2D<float> const& pt) {
//                    float ret = this->getEnergy(pAcq, pt);
//
//                    fout_trials << pt.x() << "," << pt.y() << "," << pt.theta() << "," << ret << endl;
//
//                    return ret;
//                },
//                [&camDir_rob, &posU](AbsPos2D<float> const& pt, int iter) {
//                    const float fact = pow(2, -(iter + 1));
//
//                    std::array<AbsPos2D<float>, 2> ret;
//
//                    // compute dx/dy of robot for a corresponding rotation of image
//                    float dti = posU.theta * fact;
//                    Vector2D<float> d_rob(camDir_rob - camDir_rob.rotated(dti));
//                    Vector2D<float> d_pg(d_rob.rotated(pt.theta()));
//
//                    ret[0] = std::move(AbsPos2D<float>(pt.x() + d_pg.x, pt.y() + d_pg.y, pt.theta() + dti));
//
//                    // compute dx/dy of robot for a corresponding rotation of image
//                    dti = -posU.theta * fact;
//                    d_rob = std::move(camDir_rob - camDir_rob.rotated(dti));
//                    d_pg = std::move(d_rob.rotated(pt.theta()));
//
//                    ret[1] = std::move(AbsPos2D<float>(pt.x() + d_pg.x, pt.y() + d_pg.y, pt.theta() + dti));
//
//                    return ret;
//                });
//
//        cout << "  endpos: " << currPos.x() << ", " << currPos.y() << ", " << currPos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, currPos) << endl;

//        handleStep(pAcq, currPos, iter);
    }
    perf.endOfStep("ProcAbsPosNTree2::optim");

    fout_trials.close();

    handleEnd(pAcq, currPos);
}
