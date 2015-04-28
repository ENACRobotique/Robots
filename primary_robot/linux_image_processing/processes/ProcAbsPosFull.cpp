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
#include <processes/ProcAbsPos.h>
#include <processes/ProcAbsPosFull.h>
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

#define COMP_TESTPOINTS
#define COMP_SIMULATED
//#define COMP_PLAYGROUND
//#define COMP_HSV
//#define HSV_TO_HGRAY
//#define HSV_TO_VGRAY

#define WRITE_IMAGES
//#define SHOW_IMAGES

void ProcAbsPosFull::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) {
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


    const Vector2D<float> camDir_rob(pAcq.cam2plane(acq->getCam()->getCenter()));

    ofstream fout_trials("out_trials.csv");
    fout_trials << "x,y,theta,E" << endl;

    float min_e = FLT_MAX;
    AbsPos2D<float> endPos(pos);

    for(float x = pos.x() - sqrt(posU.a_var); x < pos.x() + sqrt(posU.a_var); x += 1.f) {
        for(float y = pos.y() - sqrt(posU.b_var); y < pos.y() + sqrt(posU.b_var); y += 1.f) {
            for(float t = pos.theta() - posU.theta; t < pos.theta() + posU.theta; t += 1.f * M_PI / 180.f) {
                AbsPos2D<float> currPos(x, y, t);
                float ret = getEnergy(pAcq, currPos);

                if(ret < min_e){
                    min_e = ret;
                    endPos = std::move(currPos);
                }

                fout_trials << x << "," << y << "," << t << "," << ret << endl;
            }
        }

    }
    fout_trials.close();
    return;

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

    Mat pg_proj = getPgWithSimulatedAt(pAcq, endPos);
    imwrite("pg_simu.png", pg_proj);
}
