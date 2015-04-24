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



    // get default images
    pg = imread("Images/Table2015.png");//"simu/src_colors.png");
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
    Mat im2 = acq->getMat(RGB).clone();

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

    Point2i camCornersPoints[4];
    float factor = 4; // (px/cm)

    Mat camEdges[4];
    for (int i = 0; i < 4; i++) {
        camCornersPoints[i] = Point2i(camCorners[i].at<float>(0) * factor, (200. - camCorners[i].at<float>(1)) * factor);

        camEdges[i] = camCorners[(i + 1) % 4] - camCorners[i];
    }

    Mat pg_fov = pg.clone();
    Scalar color_fov(0, 0, 0);
    for (int i = 0; i < 4; i++) {
        line(pg_fov, camCornersPoints[i], camCornersPoints[(i+1)%4], color_fov, 4);
    }

    int nb = 0;
    for (const TestPoint& tp : staticTP) {
        cv::Mat tp_pos = tp.getPos();

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

        pg_fov.at<Vec3b>(Point2i(round(tp_pos.at<float>(0) * factor), round((200. - tp_pos.at<float>(1)) * factor))) = Vec3b(255, 255, 255);

        nb++;

        cv::Mat tp_cmRob = tr_pg2rob.transformLinPos(tp_pos);
        cv::Mat tp_pxCam = pAcq.plane2cam(tp_cmRob);

        int x = int(round(tp_pxCam.at<float>(0)));
        int y = int(round(tp_pxCam.at<float>(1)));

        assert(x >= 0 && x < cam->getSize().width);
        assert(y >= 0 && y < cam->getSize().height);

        im2.at<Vec3b>(y, x) = Vec3b(255, 255, 255);

        float hue = float(im.at<Vec3b>(y, x)[0]) / 255.;

        assert(hue >= 0. && hue <= 1.);

        E += tp.getCost(hue);
    }

    imshow("pg", pg_fov);
    imshow("testpoints", im2);

    cout << "nb=" << nb << endl;
    cout << "E =" << E << endl;

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
