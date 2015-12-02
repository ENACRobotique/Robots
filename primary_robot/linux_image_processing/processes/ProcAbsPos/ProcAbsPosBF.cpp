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
#include "ProcAbsPos.h"
#include "ProcAbsPosBF.h"
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

void ProcAbsPosBF::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
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

    float min_e = FLT_MAX;
    AbsPos2D<float> endPos(pos);

    perf.endOfStep("ProcAbsPosBF::prepare optim");

    for(float x = pos.x() - sqrt(posU.a_var); x < pos.x() + sqrt(posU.a_var); x += 1.f) {
        cout << (x - (pos.x() - sqrt(posU.a_var)))/(2.f * sqrt(posU.a_var)) * 100.f << " %" << endl;

        for(float y = pos.y() - sqrt(posU.b_var); y < pos.y() + sqrt(posU.b_var); y += 1.f) {
            for(float t = pos.theta() - posU.theta; t < pos.theta() + posU.theta; t += 1.f * M_PI / 180.f) {
                AbsPos2D<float> currPos(x, y, t);
                float ret = getEnergy(pAcq, currPos);

                if(ret < min_e){
                    min_e = ret;
                    endPos = std::move(currPos);

                    cout << "  newpos: " << endPos.x() << ", " << endPos.y() << ", " << endPos.theta() * 180. / M_PI << ", E=" << ret << endl;
                }

                fout_trials << x << "," << y << "," << t << "," << ret << endl;
            }
        }
    }

    perf.endOfStep("ProcAbsPosBF::optim");

    fout_trials.close();

    cout << "  endpos: " << endPos.x() << ", " << endPos.y() << ", " << endPos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, endPos) << endl;

    handleEnd(pAcq, endPos);
}
