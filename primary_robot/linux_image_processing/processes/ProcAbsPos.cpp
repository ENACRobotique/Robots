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

        staticTP.push_back(TestPoint((Mat_<float>(2, 1) << x, y), hue, 1.));
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

    Transform2D<float> tr_pg2rob(robPos.getTransform());
    Transform2D<float> tr_rob2pg = tr_pg2rob.getReverse();
    cv::Mat mat_pg2rob = tr_pg2rob.getMatrix();
    cv::Mat mat_rob2pg = tr_rob2pg.getMatrix();

    // get corners and edges of cam fov projected on playground
    Mat camCorners[4];
    camCorners[0] = tr_rob2pg.transformLinPos(pAcq.cam2plane(cam->getTopLeft()));
    camCorners[1] = tr_rob2pg.transformLinPos(pAcq.cam2plane(cam->getTopRight()));
    camCorners[2] = tr_rob2pg.transformLinPos(pAcq.cam2plane(cam->getBottomRight()));
    camCorners[3] = tr_rob2pg.transformLinPos(pAcq.cam2plane(cam->getBottomLeft()));
    Mat camEdges[4];
    for (int i = 0; i < 4; i++) {
        camEdges[i] = camCorners[(i + 1) % 4] - camCorners[i];
    }

    int nb = 0;
    for (const TestPoint& tp : staticTP) {
        int i;
        for (i = 0; i < 4; i++) {
            Mat vi = tp.getPos() - camCorners[i];

            double cross = vi.at<float>(0) * camEdges[i].at<float>(1) -
                    vi.at<float>(1) * camEdges[i].at<float>(0);
            if (cross < 0) {
                break;
            }
        }
        if (i < 4) {
            continue;
        }

        nb++;

//        Pt tp_cmRob = tr_pg2rob.transformLinPos(tp.getPos());
//        Pt tp_pxCam = pAcq->plane2Cam(tp_cmRob);
//
//        const Scalar& px = im.at<Scalar>(tp_pxCam.x, tp_pxCam.y);
//
//        E += tp.getCost(px(0));
    }

    cout << "nb=" << nb << endl;

//    for(const TestPoint& tp : posDependentTP){
//        Pt tp_cmRob = tr_pg2rob.transformLinPos(tp.getPos());
//        Pt tp_pxCam = pAcq->plane2Cam(tp_cmRob);
//
//        const Scalar& px = im.at<Scalar>(tp_pxCam.x, tp_pxCam.y);
//
//        E += tp.getCost(px(0));
//    }

    return E;
}

void ProcAbsPos::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
    assert(acqList.size() == 1);

    Acq* acq = acqList.front();
    Mat im = acq->getMat(HSV);

    imshow("hsv", im);

    static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
    ProjAcq pAcq = acq->projectOnPlane(pl);

//    Point3D<float> pt3d = pAcq.cam2plane(Point2D<float>(639., 479./2));
//    cout << "pt3d=" << pt3d << endl;
//
//    Point2D<float> pt2d = pAcq.plane2cam(pt3d);
//    cout << "pt2d=" << pt2d << endl;

    Pos currPos = pos;
    float prevE = getEnergy(pAcq, currPos);

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
