/*
 * ProcAbsPos.cpp
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <performance.hpp>
#include <processes/ProcAbsPos.h>
#include <Plane3D.h>
#include <tools/Cam.h>
#include <tools/Image.h>
#include <tools/neldermead.h>
#include <tools/ProjAcq.h>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>

using namespace std;
using namespace cv;

//#define SHOW_TESTPOINTS
#define SHOW_SIMULATED
#define HSV_TO_HGRAY
//#define HSV_TO_VGRAY
//#define SHOW_PLAYGROUND
#define SHOW_HSV

#define WRITE_IMAGES

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
        float x, y, hue, sat, val, prob;
        char del;

        s >> x >> del >> y >> del >> hue >> del >> sat >> del >> val >> del >> prob;

        assert(del == ',');
        assert(hue >= 0 && hue <= 1);
        assert(sat >= 0 && sat <= 1);
        assert(val >= 0 && val <= 1);
        assert(prob >= 0 && prob <= 1);

        staticTP.push_back(TestPoint((Mat_<float>(2, 1) << x, y), hue, sat, val, 10. * prob));
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

    return {int(round(p.at<float>(0) * factor)) + 29, int(round((200 - p.at<float>(1)) * factor)) + 29};
}

template<typename T>
Point2i getInPGIm(Point_<T> p) {
    float factor = 4; // (px/cm)

    return {int(round(p.x * factor)) + 29, int(round((200 - p.y) * factor)) + 29};
}

template<typename T>
Point2i getInPGIm(Point3_<T> p) {
    float factor = 4; // (px/cm)

    return {int(round(p.x * factor)) + 29, int(round((200 - p.y) * factor)) + 29};
}

Mat ProcAbsPos::getSimulatedAt(ProjAcq& pAcq, const Pos& robPos) const {
    Transform2D<float> tr_rob2pg = robPos.getTransform().getReverse();

    return getSimulatedAt(pAcq, tr_rob2pg);
}

Mat ProcAbsPos::getSimulatedAt(ProjAcq& pAcq, const Transform2D<float>& tr_rob2pg) const {
    // simulate acquisition
    Mat im3 = pAcq.getAcq()->getMat(BGR).clone();
    for (Mat_<Vec3b>::iterator it = im3.begin<Vec3b>(); it != im3.end<Vec3b>();
            it++) {
        Point2i pos_pg = getInPGIm(tr_rob2pg.transformLinPos(pAcq.cam2plane(it.pos())));
        if (pos_pg.y < 0 || pos_pg.y >= pg.size[0] || pos_pg.x < 0 || pos_pg.x >= pg.size[1]) {
            (*it) = Vec3b(0, 0, 0);
        }
        else {
            (*it) = pg.at<Vec3b>(pos_pg);
        }
    }

    return im3;
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

//#ifdef SHOW_SIMULATED
//    // simulate acquisition
//    Mat im3 = getSimulatedAt(pAcq, tr_rob2pg);
//    imshow("im3", im3);
//
//#ifdef SHOW_HSV
//    // show simulated acquisition in hsv
//    Mat im3_hsv = im3.clone();
//    cvtColor(im3, im3_hsv, COLOR_BGR2HSV);
//#ifdef HSV_TO_HGRAY
//    for(Mat_<Vec3b>::iterator it = im3_hsv.begin<Vec3b>(); it != im3_hsv.end<Vec3b>(); it++) {
//        (*it)[1] = (*it)[0];
//        (*it)[2] = (*it)[0];
//    }
//#elif defined(HSV_TO_VGRAY)
//    for(Mat_<Vec3b>::iterator it = im3_hsv.begin<Vec3b>(); it != im3_hsv.end<Vec3b>(); it++) {
//        (*it)[0] = (*it)[2];
//        (*it)[1] = (*it)[2];
//    }
//#endif
//#ifdef WRITE_IMAGES
//    imwrite("im3_hsv.png", im3_hsv);
//#endif
//    imshow("im3_hsv", im3_hsv);
//#endif
//#endif

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
        float sat = float(im.at<Vec3b>(y, x)[1]) / 255.;
        float val = float(im.at<Vec3b>(y, x)[2]) / 255.;

//        assert(hue >= 0. && hue <= 1.);
//        assert(255*hue == im.at<Vec3b>(y, x)[0]);

        // get cost of testpoint given this hue
        E += tp.getCost(hue, sat, val);
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

float ProcAbsPos::f(ProcAbsPos& p, ProjAcq& pAcq, AbsPos2D<float> const& pt, int iter) {
    float enr = p.getEnergy(pAcq, pt);
    float rnd = iter >= 0 ? enr * 1.f * p.getRand() / log(3.f + iter) : 0.f;
    float ret = enr + rnd;

// cout << "f(pt = " << pt << ", iter = " << iter << ") = " << enr << " + " << rnd / enr * 100.f << " %" << endl;

    return ret;
}

void ProcAbsPos::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
    assert(acqList.size() == 1);

    Acq* acq = acqList.front();

#ifdef WRITE_IMAGES
    imwrite("rgb.png", acq->getMat(BGR));
#endif

#ifdef SHOW_HSV
    Mat im_hsv = acq->getMat(HSV);

#ifdef HSV_TO_HGRAY
    for (Mat_<Vec3b>::iterator it = im_hsv.begin<Vec3b>();
            it != im_hsv.end<Vec3b>(); it++) {
        (*it)[1] = (*it)[0];
        (*it)[2] = (*it)[0];
    }
#elif defined(HSV_TO_VGRAY)
    for (Mat_<Vec3b>::iterator it = im_hsv.begin<Vec3b>();
            it != im_hsv.end<Vec3b>(); it++) {
        (*it)[0] = (*it)[2];
        (*it)[1] = (*it)[2];
    }
#endif

#ifdef WRITE_IMAGES
    imwrite("hsv.png", im_hsv);
#endif

    imshow("hsv", im_hsv);
#endif

    Perf& perf = Perf::getPerf();

    static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
    ProjAcq pAcq = acq->projectOnPlane(pl);

    cout << "  begpos: " << pos.x() << ", " << pos.y() << ", " << pos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, pos) << endl;

    float du = 10;
    float dv = 10;
    float dr = 5;
    float cm2rad = 1.f / 50.f;

    for (int i = 0; i < 3; i++) {
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
                                dr * cm2rad * getRand()),
                AbsPos2D<float>(
                        pos.x() - st * dv +
                                dr * getRand(),
                        pos.y() + ct * dv +
                                dr * getRand(),
                        pos.theta() +
                                dr * cm2rad * getRand()),
                AbsPos2D<float>(
                        pos.x() + ct * du +
                                dr * getRand(),
                        pos.y() + st * du +
                                dr * getRand(),
                        pos.theta() + du * cm2rad +
                                dr * cm2rad * getRand()),
                AbsPos2D<float>(
                        pos.x() + st * dv +
                                dr * getRand(),
                        pos.y() - ct * dv +
                                dr * getRand(),
                        pos.theta() - du * cm2rad +
                                dr * cm2rad * getRand()),
        };

        cout << "input simplex:" << endl;
        for (AbsPos2D<float> const& v : simplex) {
            cout << v.x() << " " << v.y() << " " << v.theta() << endl;
        }

        function<float(AbsPos2D<float> const&, int)> bf = std::bind(f, ref(*this), ref(pAcq), placeholders::_1, placeholders::_2);

        perf.endOfStep("ProcAbsPos::setting up optim");

        AbsPos2D<float> endPos = neldermead<float, AbsPos2D<float>, 3>(simplex, bf, 0, 20);

        cout << "output simplex:" << endl;
        for (AbsPos2D<float> const& v : simplex) {
            cout << v.x() << " " << v.y() << " " << v.theta() << endl;
        }

        cout << "  endpos: " << endPos.x() << ", " << endPos.y() << ", " << endPos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, endPos) << endl;

        perf.endOfStep("ProcAbsPos::optim (neldermead #" + to_string(i) + ")");

#ifdef SHOW_SIMULATED
        // simulate acquisition
        Mat im3 = getSimulatedAt(pAcq, endPos);
        string bn("im3-" + to_string(i));
#ifdef WRITE_IMAGES
        imwrite(bn + ".png", im3);
#endif
        imshow(bn, im3);

#ifdef SHOW_HSV
        // show simulated acquisition in hsv
        Mat im3_hsv = im3.clone();
        cvtColor(im3, im3_hsv, COLOR_BGR2HSV);
#ifdef HSV_TO_HGRAY
        for (Mat_<Vec3b>::iterator it = im3_hsv.begin<Vec3b>();
                it != im3_hsv.end<Vec3b>(); it++) {
            (*it)[1] = (*it)[0];
            (*it)[2] = (*it)[0];
        }
#elif defined(HSV_TO_VGRAY)
        for(Mat_<Vec3b>::iterator it = im3_hsv.begin<Vec3b>(); it != im3_hsv.end<Vec3b>(); it++) {
            (*it)[0] = (*it)[2];
            (*it)[1] = (*it)[2];
        }
#endif
#ifdef WRITE_IMAGES
        imwrite(bn + "_hsv.png", im3_hsv);
#endif
        imshow(bn + "_hsv", im3_hsv);

        Mat diff_hsv = im_hsv - im3_hsv;
#ifdef WRITE_IMAGES
        imwrite(bn + "_diff_hsv.png", diff_hsv);
#endif
        imshow(bn + "_diff_hsv", diff_hsv);
#endif
#endif
    }

}
