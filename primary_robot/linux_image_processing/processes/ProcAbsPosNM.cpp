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
#include <processes/ProcAbsPosNM.h>
#include <Plane3D.h>
#include <tools/AbsPos2D.h>
#include <tools/Acq.h>
#include <tools/Cam.h>
#include <tools/Image.h>
#include <tools/neldermead.h>
#include <Transform2D.h>
#include <Vector2D.h>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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

void ProcAbsPosNM::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
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


#ifdef COMP_SIMULATED
    // simulate acquisition
    Mat _im3 = getSimulatedAt(pAcq, pos);
    string _bn("_im3-sa");
#ifdef WRITE_IMAGES
    imwrite(_bn + ".png", _im3);
#endif /* WRITE_IMAGES */
#ifdef SHOW_IMAGES
    imshow(bn, _im3);
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

        perf.endOfStep("ProcAbsPos::setting up optim");

        AbsPos2D<float> endPos = neldermead<float, AbsPos2D<float>, 3>(simplex, [this, &pAcq](AbsPos2D<float> const& pt, int iter){
            float enr = this->getEnergy(pAcq, pt);
            float rnd = iter >= 0 ? enr * 1.f * this->getRand() / log(3.f + iter) : 0.f;
            float ret = enr + rnd;

        // cout << "f(pt = " << pt << ", iter = " << iter << ") = " << enr << " + " << rnd / enr * 100.f << " %" << endl;

            return ret;
        }, 0, 30);


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

        Mat pg_proj = getPgWithSimulatedAt(pAcq, endPos);
        imwrite("pg_simu-" + to_string(i) + ".png", pg_proj);
    }

}
