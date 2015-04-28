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
#include <tools/simulated_annealing.h>
#include <tools/ProjAcq.h>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>

using namespace std;
using namespace cv;

#define COMP_TESTPOINTS
#define COMP_SIMULATED
//#define COMP_PLAYGROUND
//#define COMP_HSV
//#define HSV_TO_HGRAY
//#define HSV_TO_VGRAY

#define WRITE_IMAGES
//#define SHOW_IMAGES

ProcAbsPos::ProcAbsPos(Cam* c, const string& staticTestPointFile)
        {
    camList.push_back(c);

    ifstream infile(staticTestPointFile);
    string line;
    while (getline(infile, line)) {
        istringstream s(line);
        float x, y /* (cm) */, hue = -1, sat = -1, val = -1,
                dist = -1 /* (cm) */, dens = -1 /* (cm^-2) */;
        char del;

        s >> x >> del >> y >> del >> hue >> del >> sat >> del >> val >> del >> dist >> del >> dens;

        assert(del == ',');
        assert(hue >= 0 && hue <= 1);
        assert(sat >= 0 && sat <= 1);
        assert(val >= 0 && val <= 1);
        assert(dist >= 0 && dist <= 100);
        assert(dens > 0 && dens <= 10);

        staticTP.push_back(TestPoint(x, y, hue, sat, val, 1.f / dens));
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

void ProcAbsPos::addTestPointsAtTo(Mat& im, ProjAcq& pAcq, const Transform2D<float>& tr_rob2pg) const {
    Transform2D<float> tr_pg2rob = tr_rob2pg.getReverse();
    Cam const* cam = pAcq.getAcq()->getCam();

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

    Mat camEdges[4];
    for (int i = 0; i < 4; i++) {
        camEdges[i] = camCorners[(i + 1) % 4] - camCorners[i];
    }

    for (TestPoint const& tp : staticTP) {
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

        // get position of testpoint in original camera image
        cv::Mat tp_cmRob = tr_pg2rob.transformLinPos(tp_pos);
        cv::Mat tp_pxCam = pAcq.plane2cam(tp_cmRob);

        int x = int(round(tp_pxCam.at<float>(0)));
        int y = int(round(tp_pxCam.at<float>(1)));

        assert(x >= 0 && x < cam->getSize().width);
        assert(y >= 0 && y < cam->getSize().height);

        // draw testpoint position in camera frame
        im.at<Vec3b>(y, x) = Vec3b(255, 255, 255);
    }
}

Mat ProcAbsPos::getTestPointsAt(ProjAcq& pAcq, const Transform2D<float>& tr_rob2pg) const {
    Mat im3 = getSimulatedAt(pAcq, tr_rob2pg);

    addTestPointsAtTo(im3, pAcq, tr_rob2pg);

    return im3;
}

float ProcAbsPos::getEnergy(ProjAcq& pAcq, const Pos& robPos) {
    vector<TestPoint> posDependentTP = getPosDependentTP(robPos);

    float E = 0;

    Acq* acq = pAcq.getAcq();
    Cam const* cam = acq->getCam();
    Mat im = acq->getMat(HSV);

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

#ifdef COMP_PLAYGROUND
    Point2i camCornersPoints[4];
#endif /* COMP_PLAYGROUND */

    Mat camEdges[4];
    for (int i = 0; i < 4; i++) {
#ifdef COMP_PLAYGROUND
        camCornersPoints[i] = getInPGIm(camCorners[i]);
#endif /* COMP_PLAYGROUND */
        camEdges[i] = camCorners[(i + 1) % 4] - camCorners[i];
    }

#ifdef COMP_PLAYGROUND
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
#endif /* COMP_PLAYGROUND */

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

#ifdef COMP_PLAYGROUND
        // testpoint is in cam field of view, draw it in playground
        pg_fov.at<Vec3b>(getInPGIm(tp_pos)) = Vec3b(255, 255, 255);
#endif /* COMP_PLAYGROUND */

        // get position of testpoint in original camera image
        cv::Mat tp_cmRob = tr_pg2rob.transformLinPos(tp_pos);
        cv::Mat tp_pxCam = pAcq.plane2cam(tp_cmRob);

        int x = int(round(tp_pxCam.at<float>(0)));
        int y = int(round(tp_pxCam.at<float>(1)));

        assert(x >= 0 && x < cam->getSize().width);
        assert(y >= 0 && y < cam->getSize().height);

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

#ifdef COMP_PLAYGROUND
#ifdef SHOW_IMAGES
    imshow("pg", pg_fov);
#endif /* SHOW_IMAGES */
#ifdef WRITE_IMAGES
    imwrite("pg.png", pg_fov);
#endif /* WRITE_IMAGES */
#endif /* COMP_PLAYGROUND */

    return E;
}

template<typename T>
inline T clamp(T v, T m, T M) { return max(m, min(v, M)); }

void ProcAbsPos::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
    assert(acqList.size() == 1);

    Acq* acq = acqList.front();

    Mat rgb = acq->getMat(BGR);
#ifdef WRITE_IMAGES
    imwrite("rgb.png", rgb);
#endif

#ifdef COMP_HSV
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
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow("hsv", im_hsv);
#endif /* SHOW_IMAGES */
#endif /* COMP_HSV */

    Perf& perf = Perf::getPerf();

    static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
    ProjAcq pAcq = acq->projectOnPlane(pl);

    cout << "  begpos: " << pos.x() << ", " << pos.y() << ", " << pos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, pos) << endl;

#if 1






#ifdef COMP_SIMULATED
    // simulate acquisition
    Mat _im3 = getSimulatedAt(pAcq, pos);
    string _bn("_im3-sa");
#ifdef WRITE_IMAGES
    imwrite(_bn + ".png", _im3);
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn, im3);
#endif /* SHOW_IMAGES */

#ifdef COMP_HSV
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
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn + "_hsv", im3_hsv);
#endif /* SHOW_IMAGES */

    Mat diff_hsv = im_hsv - im3_hsv;
#ifdef WRITE_IMAGES
    imwrite(bn + "_diff_hsv.png", diff_hsv);
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn + "_diff_hsv", diff_hsv);
#endif /* SHOW_IMAGES */
#endif /* COMP_HSV */

#ifdef COMP_TESTPOINTS
    Mat _im3_tp = getTestPointsAt(pAcq, pos.getTransform().getReverse());
    _bn = "_im3_tp-sa";
#ifdef WRITE_IMAGES
    imwrite(_bn + ".png", _im3_tp);
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn, im3_tp);
#endif /* SHOW_IMAGES */
#endif /* COMP_TESTPOINTS */

#endif /* COMP_SIMULATED */

#ifdef COMP_TESTPOINTS
    Mat _rgb_tp = rgb.clone();
    addTestPointsAtTo(_rgb_tp, pAcq, pos.getTransform().getReverse());
    _bn = "_im_tp-sa";
#ifdef WRITE_IMAGES
    imwrite(_bn + ".png", _rgb_tp);
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn, rgb_tp);
#endif /* SHOW_IMAGES */
#endif /* COMP_TESTPOINTS */









    const Vector2D<float> camDir_rob(pAcq.cam2plane(acq->getCam()->getCenter()));
    const float cm2rad = 1.f/camDir_rob.norm();

    ofstream fout_trials("out_trials.csv");
    fout_trials << "x,y,theta,E" << endl;

    AbsPos2D<float> endPos = simulated_annealing<AbsPos2D<float>, int, float>(pos, 30.f, 0.96f, 150, 12,
            [this, &pAcq, &fout_trials](AbsPos2D<float> const& pt) { // get energy
                float ret = this->getEnergy(pAcq, pt);

                fout_trials << pt.x() << "," << pt.y() << "," << pt.theta() << "," << ret << endl;

                return ret;
            },
            [this, &camDir_rob, &cm2rad](AbsPos2D<float> const& curr) { // get neighbor (4 terms: deltaX, deltaY, deltaTheta@Robot, deltaTheta@Image)
                constexpr float dr = 3.f;

                // compute dx/dy of robot for a corresponding rotation of image
                float dti = dr * cm2rad * this->getRand() / 2.f;
                Vector2D<float> d_rob(std::move(camDir_rob - camDir_rob.rotated(dti)));
                Vector2D<float> d_pg(std::move(d_rob.rotated(curr.theta())));

                float nx = clamp(curr.x() + dr * this->getRand() + d_pg.x, 20.f, 280.f);
                float ny = clamp(curr.y() + dr * this->getRand() + d_pg.y, 20.f, 180.f);
                float nt = curr.theta() + dr * cm2rad * this->getRand() + dti;

                return AbsPos2D<float>(nx, ny, nt);
            });

    fout_trials.close();

    cout << "  endpos: " << endPos.x() << ", " << endPos.y() << ", " << endPos.theta() * 180. / M_PI << ", E=" << getEnergy(pAcq, endPos) << endl;

    perf.endOfStep("ProcAbsPos::optim (simulated annealing)");

#ifdef COMP_SIMULATED
    // simulate acquisition
    Mat im3 = getSimulatedAt(pAcq, endPos);
    string bn("im3-sa");
#ifdef WRITE_IMAGES
    imwrite(bn + ".png", im3);
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn, im3);
#endif /* SHOW_IMAGES */

#ifdef COMP_HSV
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
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn + "_hsv", im3_hsv);
#endif /* SHOW_IMAGES */

    Mat diff_hsv = im_hsv - im3_hsv;
#ifdef WRITE_IMAGES
    imwrite(bn + "_diff_hsv.png", diff_hsv);
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn + "_diff_hsv", diff_hsv);
#endif /* SHOW_IMAGES */
#endif /* COMP_HSV */

#ifdef COMP_TESTPOINTS
    Mat im3_tp = getTestPointsAt(pAcq, endPos.getTransform().getReverse());
    bn = "im3_tp-sa";
#ifdef WRITE_IMAGES
    imwrite(bn + ".png", im3_tp);
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn, im3_tp);
#endif /* SHOW_IMAGES */
#endif /* COMP_TESTPOINTS */

#endif /* COMP_SIMULATED */

#ifdef COMP_TESTPOINTS
    Mat rgb_tp = rgb.clone();
    addTestPointsAtTo(rgb_tp, pAcq, endPos.getTransform().getReverse());
    bn = "im_tp-sa";
#ifdef WRITE_IMAGES
    imwrite(bn + ".png", rgb_tp);
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn, rgb_tp);
#endif /* SHOW_IMAGES */
#endif /* COMP_TESTPOINTS */

#else
    float du = 10;
    float dv = 10;
    float dr = 0;
    float cm2rad = 1.f / 40.f;

    Transform2D<float> tr_pg2rob = pos.getTransform();
    Transform2D<float> tr_rob2pg = tr_rob2pg.getReverse();
    Vector2D<float> camDir(pAcq.cam2plane(acq->getCam()->getCenter()));

    cout << "camDir: " << camDir << endl;

    for (int i = 0; i < 1; i++) {
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
                    dr * cm2rad * getRand(),
                    camDir),
            AbsPos2D<float>(
                    pos.x() - st * dv +
                    dr * getRand(),
                    pos.y() + ct * dv +
                    dr * getRand(),
                    pos.theta() +
                    dr * cm2rad * getRand(),
                    camDir),
            AbsPos2D<float>(
                    pos.x() + ct * du +
                    dr * getRand(),
                    pos.y() + st * du +
                    dr * getRand(),
                    pos.theta() + du * cm2rad +
                    dr * cm2rad * getRand(),
                    camDir),
            AbsPos2D<float>(
                    pos.x() + st * dv +
                    dr * getRand(),
                    pos.y() - ct * dv +
                    dr * getRand(),
                    pos.theta() - du * cm2rad +
                    dr * cm2rad * getRand(),
                    camDir),
        };

        cout << "input simplex:" << endl;
        for (AbsPos2D<float> const& v : simplex) {
            cout << v.x() << " " << v.y() << " " << v.theta() << endl;
        }

        function<float(AbsPos2D<float> const&, int)> bf = std::bind(f, ref(*this), ref(pAcq), placeholders::_1, placeholders::_2);

        perf.endOfStep("ProcAbsPos::setting up optim");

        AbsPos2D<float> endPos = neldermead<float, AbsPos2D<float>, 3>(simplex, bf, 0, 30);

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
#endif

}
