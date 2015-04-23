/*
 * ProcAbsPos.cpp
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <processes/ProcAbsPos.h>
#include <Plane3D.h>
#include <Point2D.h>
#include <Transform2D.h>
#include <tools/Image.h>
#include <tools/Position2D.h>
#include <tools/ProjAcq.h>
#include <cassert>
#include <fstream>

using namespace std;
using namespace cv;

ProcAbsPos::ProcAbsPos(Cam* c, const string& staticTestPointFile)
        {
    camList.push_back(c);

    ifstream infile(staticTestPointFile);
    string line;
    while (getline(infile, line)) {
        istringstream s(line);
        float x, y, hue;
        char del;

        s >> x >> del >> y >> del >> hue;

        assert(del == ',');

        staticTP.push_back(TestPoint(Pt(x, y), hue, 1.));
    }
    infile.close();

    cout << "Read " << staticTP.size() << " testpoints from file \"" << staticTestPointFile << "\"" << endl;
}

ProcAbsPos::~ProcAbsPos()
{
}

vector<TestPoint> ProcAbsPos::getPosDependentTP(const Pos& robPos) {
    // TODO
    return vector<TestPoint>();
}

float ProcAbsPos::getEnergy(ProjAcq& pAcq, const Pos& robPos) {
    vector<TestPoint> posDependentTP = getPosDependentTP(robPos);

    float E = 0;

    Acq* acq = pAcq.getAcq();
    Cam const* cam = acq->getCam();
    Mat im = acq->getMat(HSV);

    Transform2D<float> tr_pg2rob = robPos.getTransform();

    Point3D<float> camTL = pAcq.cam2plane(cam->getTopLeft());
    Point3D<float> camTR = pAcq.cam2plane(cam->getTopRight());
    Point3D<float> camBR = pAcq.cam2plane(cam->getBottomRight());
    Point3D<float> camBL = pAcq.cam2plane(cam->getBottomLeft());

    cout << "TL: " << camTL << endl;
    cout << "TR: " << camTR << endl;
    cout << "BR: " << camBR << endl;
    cout << "BL: " << camBL << endl;

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

    static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
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
