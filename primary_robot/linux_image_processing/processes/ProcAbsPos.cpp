/*
 * ProcAbsPos.cpp
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#include <opencv2/highgui/highgui.hpp>
#include <processes/ProcAbsPos.h>
#include <Point2D.h>
#include <tools/Image.h>
#include <tools/ProjAcq.h>
#include <cassert>

using namespace std;
using namespace cv;

ProcAbsPos::ProcAbsPos(Cam* c, const string& staticTestPointFile)
        {
    camList.push_back(c);

    // TODO read input file and fill staticTP vector

    staticTP.push_back(TestPoint(Pt(10, 10), 0., 1.));
}

ProcAbsPos::~ProcAbsPos()
{
}

vector<TestPoint> ProcAbsPos::getPosDependentTP(const Pos& robPos){
    // TODO
    return vector<TestPoint>();
}

//float ProcAbsPos::getEnergy(ProjAcq& pAcq, const Pos& robPos){
//    vector<TestPoint> posDependentTP = getPosDependentTP(robPos);
//
//    float E = 0;
//
//    Acq* acq = pAcq.getAcq();
//    const Mat& im = acq->getMat(HSV);
//    Transform2D tr_pg2rob = robPos.getTransform();
//
//    for(const TestPoint& tp : staticTP){
//        Pt tp_cmRob = tr_pg2rob.transformLinPos(tp.getPos());
//        Pt tp_pxCam = pAcq->plane2Cam(tp_cmRob);
//
//        const Scalar& px = im.at<Scalar>(tp_pxCam.x, tp_pxCam.y);
//
//        E += tp.getCost(px(0));
//    }
//
//    for(const TestPoint& tp : posDependentTP){
//        Pt tp_cmRob = tr_pg2rob.transformLinPos(tp.getPos());
//        Pt tp_pxCam = pAcq->plane2Cam(tp_cmRob);
//
//        const Scalar& px = im.at<Scalar>(tp_pxCam.x, tp_pxCam.y);
//
//        E += tp.getCost(px(0));
//    }
//
//    return E;
//}

void ProcAbsPos::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
    assert(acqList.size() == 1);

    Acq* acq = acqList.front();
    Mat im = acq->getMat(HSV);

    imshow("hsv", im);

    Plane3D<float> pl({0, 0, 0}, {0, 0, 1}); // build a plane with a point and a normal
    ProjAcq pAcq = acq->projectOnPlane(pl);

//
//    float prevE = getEnergy(pAcq, currPos);
//    Pos currPos = pos;
//    float T = ...;
//    float alpha = 0.9...;
//    do{
//        Pos testPos = currPos + rand();
//        float E = getEnergy(pAcq, testPos);
//
//        if(E-prevE ... > ...){
//            currPos = testPos;
//        }
//
//        T *= alpha;
//        prevE = E;
//    }while(1);
}
