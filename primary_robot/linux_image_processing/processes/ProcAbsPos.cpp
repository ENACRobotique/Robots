/*
 * ProcAbsPos.cpp
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <processes/ProcAbsPos.h>
#include <Plane3D.h>
#include <Point2D.h>
#include <tools/Cam.h>
#include <tools/Image.h>
#include <tools/Position2D.h>
#include <tools/ProjAcq.h>
#include <Transform2D.h>
#include <cassert>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <random>
#include "performance.hpp"

using namespace std;
using namespace cv;

//#define SHOW_TESTPOINTS
//#define SHOW_SIMULATED
//#define HSV_TO_HGRAY
//#define SHOW_PLAYGROUND
//#define SHOW_HSV

/**
 * H
 * 0°-360° rouge
 * 60° jaune
 * 120° vert
 * 180° cyan
 * 240° bleu
 * 300° magenta
 */

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
        assert(hue >= 0 && hue <= 1);

        staticTP.push_back(TestPoint((Mat_<float>(2, 1) << x, y), hue, 1.));
    }
    infile.close();

    cout << "Read " << staticTP.size() << " testpoints from file \"" << staticTestPointFile << "\"" << endl;

    // get default images
    pg = imread("simu/src_colors.png");
}

ProcAbsPos::~ProcAbsPos()
{
}

vector<TestPoint> ProcAbsPos::getPosDependentTP(const Pos& robPos) {
    // TODO
    return vector<TestPoint>();
}

Point2i getInPGIm(Mat p) {
    float factor = 4; // (px/cm)

    return {int(round(p.at<float>(0) * factor)) + 9, int(round((200 - p.at<float>(1)) * factor)) + 9};
}

template<typename T>
Point2i getInPGIm(Point_<T> p) {
    float factor = 4; // (px/cm)

    return {int(round(p.x * factor)) + 9, int(round((200 - p.y) * factor)) + 9};
}

template<typename T>
Point2i getInPGIm(Point3_<T> p) {
    float factor = 4; // (px/cm)

    return {int(round(p.x * factor)) + 9, int(round((200 - p.y) * factor)) + 9};
}

float ProcAbsPos::getEnergy(ProjAcq& pAcq, const Pos& robPos) {
    vector<TestPoint> posDependentTP = getPosDependentTP(robPos);

    float E = 0;

    Acq* acq = pAcq.getAcq();
    Cam const* cam = acq->getCam();
    Mat im = acq->getMat(HSV);
#ifdef SHOW_TESTPOINTS
    Mat im2 = acq->getMat(BGR).clone();
#endif

    Transform2D<float> tr_pg2rob(robPos.getTransform());
    Transform2D<float> tr_rob2pg = tr_pg2rob.getReverse();

    // get corners and edges of cam fov projected on playground
    Mat camCorners[4];
    camCorners[0] = tr_rob2pg.transformLinPos(pAcq.cam2plane(cam->getTopLeft()));
    camCorners[1] = tr_rob2pg.transformLinPos(pAcq.cam2plane(cam->getTopRight()));
    camCorners[2] = tr_rob2pg.transformLinPos(pAcq.cam2plane(cam->getBottomRight()));
    camCorners[3] = tr_rob2pg.transformLinPos(pAcq.cam2plane(cam->getBottomLeft()));

    float xMin = MIN(MIN(camCorners[0].at<float>(0), camCorners[1].at<float>(0)), MIN(camCorners[2].at<float>(0), camCorners[3].at<float>(0)));
    float yMin = MIN(MIN(camCorners[0].at<float>(1), camCorners[1].at<float>(1)), MIN(camCorners[2].at<float>(1), camCorners[3].at<float>(1)));
    float xMax = MAX(MAX(camCorners[0].at<float>(0), camCorners[1].at<float>(0)), MAX(camCorners[2].at<float>(0), camCorners[3].at<float>(0)));
    float yMax = MAX(MAX(camCorners[0].at<float>(1), camCorners[1].at<float>(1)), MAX(camCorners[2].at<float>(1), camCorners[3].at<float>(1)));

#ifdef SHOW_PLAYGROUND
    Point2i camCornersPoints[4];
#endif

    Mat camEdges[4];
    for (int i = 0; i < 4; i++) {
#ifdef SHOW_PLAYGROUND
        camCornersPoints[i] = getInPGIm(camCorners[i]);
#endif
        camEdges[i] = camCorners[(i + 1) % 4] - camCorners[i];
    }

#ifdef SHOW_PLAYGROUND
    Mat pg_fov = pg.clone();

    // draw projected FOV
    Scalar color_fov(0, 0, 0);
    for (int i = 0; i < 4; i++) {
        line(pg_fov, camCornersPoints[i], camCornersPoints[(i + 1) % 4], color_fov, 4);
    }

    // draw robot position
    Point2i rP = getInPGIm(robPos.getLinPos());
    Point2i rP_x = getInPGIm(robPos.getLinPos() + Point2D<float>(3, 0).rotate(robPos.theta()).toCv());
    line(pg_fov, rP, rP_x, Scalar(0, 0, 255), 4);
    Point2i rP_y = getInPGIm(robPos.getLinPos() + Point2D<float>(0, 3).rotate(robPos.theta()).toCv());
    line(pg_fov, rP, rP_y, Scalar(0, 255, 0), 4);
#endif

#ifdef SHOW_SIMULATED
    // simulate acquisition
    Mat im3 = acq->getMat(BGR).clone();
    for(Mat_<Vec3b>::iterator it = im3.begin<Vec3b>(); it != im3.end<Vec3b>(); it++) {
        (*it) = pg.at<Vec3b>(getInPGIm(tr_rob2pg.transformLinPos(pAcq.cam2plane(it.pos()))));
    }
    imshow("im3", im3);

#ifdef SHOW_HSV
    // show simulated acquisition in hsv
    Mat im3_hsv = im3.clone();
    cvtColor(im3, im3_hsv, COLOR_BGR2HSV);
#ifdef HSV_TO_HGRAY
    for(Mat_<Vec3b>::iterator it = im3_hsv.begin<Vec3b>(); it != im3_hsv.end<Vec3b>(); it++) {
        (*it)[1] = (*it)[0];
        (*it)[2] = (*it)[0];
    }
#endif
    imshow("im3_hsv", im3_hsv);
#endif
#endif

    // get Energy...
    int nb = 0;
    for (TestPoint& tp : staticTP) {
        cv::Mat tp_pos = tp.getPos();

        float tp_x = tp_pos.at<float>(0);
        float tp_y = tp_pos.at<float>(1);

        if (tp_x > xMax || tp_x < xMin || tp_y > yMax || tp_y < yMin) {
            continue;
        }

        // check if testpoint seen by camera
        int i;
        for (i = 0; i < 4; i++) {
            Mat vi = tp_pos - camCorners[i];

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

#ifdef SHOW_PLAYGROUND
        // testpoint is in cam field of view, draw it in playground
        pg_fov.at<Vec3b>(getInPGIm(tp_pos)) = Vec3b(255, 255, 255);
#endif

        // get position of testpoint in original camera image
        cv::Mat tp_cmRob = tr_pg2rob.transformLinPos(tp_pos);
        cv::Mat tp_pxCam = pAcq.plane2cam(tp_cmRob);

        int x = int(round(tp_pxCam.at<float>(0)));
        int y = int(round(tp_pxCam.at<float>(1)));

        assert(x >= 0 && x < cam->getSize().width);
        assert(y >= 0 && y < cam->getSize().height);

#ifdef SHOW_TESTPOINTS
        // draw testpoint position in camera frame
        im2.at<Vec3b>(y, x) = Vec3b(255, 255, 255);
#endif

        // get hue of selected pixel
        float hue = float(im.at<Vec3b>(y, x)[0]) / 255.;

//        assert(hue >= 0. && hue <= 1.);
//        assert(255*hue == im.at<Vec3b>(y, x)[0]);

        // get cost of testpoint given this hue
        E += tp.getCost(hue);
    }

//    for(const TestPoint& tp : posDependentTP){
//        Pt tp_cmRob = tr_pg2rob.transformLinPos(tp.getPos());
//        Pt tp_pxCam = pAcq->plane2Cam(tp_cmRob);
//
//        const Scalar& px = im.at<Scalar>(tp_pxCam.x, tp_pxCam.y);
//
//        E += tp.getCost(px(0));
//    }

    E /= nb;

#ifdef SHOW_PLAYGROUND
    imshow("pg", pg_fov);
#endif
#ifdef SHOW_TESTPOINTS
    imshow("testpoints", im2);
#endif

    return E;
}

void ProcAbsPos::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
    assert(acqList.size() == 1);

    Acq* acq = acqList.front();
#ifdef SHOW_HSV
    Mat im = acq->getMat(HSV);

#ifdef HSV_TO_HGRAY
    for(Mat_<Vec3b>::iterator it = im.begin<Vec3b>(); it != im.end<Vec3b>(); it++) {
        (*it)[1] = (*it)[0];
        (*it)[2] = (*it)[0];
    }
#endif

    imshow("hsv", im);
#endif

    Perf& perf = Perf::getPerf();

    static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
    ProjAcq pAcq = acq->projectOnPlane(pl);

    Pos currPos = pos;
    float E = 0, prevE = getEnergy(pAcq, currPos);

    perf.endOfStep("begpos");

    cout << "  begpos: " << currPos.x() << ", " << currPos.y() << ", " << currPos.theta() * 180. / M_PI << ", E=" << prevE << endl;

//    float T = ...;
//    float alpha = 0.9...;
    int nb = 10;
    do {
        float dx = 5 * getRand();
        float dy = 5 * getRand();
        float dt = 5 * M_PI / 180. * getRand();

        Pos testPos(currPos.x() + dx, currPos.y() + dy, currPos.theta() + dt);
        E = getEnergy(pAcq, testPos);

        if (E < prevE) {
            currPos = testPos;
            prevE = E;

//            cout << "  newpos: " << currPos.x() << ", " << currPos.y() << ", " << currPos.theta() * 180. / M_PI << ", E=" << E << endl;
        }

        perf.endOfStep("it #" + to_string(11 - nb));

//        T *= alpha;
    } while (--nb > 0);

    cout << "  endpos: " << currPos.x() << ", " << currPos.y() << ", " << currPos.theta() * 180. / M_PI << ", E=" << prevE << endl;
}
