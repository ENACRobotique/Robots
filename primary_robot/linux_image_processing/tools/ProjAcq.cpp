#include <Point2D.h>
#include <Point3D.h>
#include <tools/Acq.h>
#include <tools/Cam.h>
#include <tools/ProjAcq.h>
#include <cstdint>
#include <map>
#include <utility>

using namespace std;
using namespace cv;

ProjAcq::ProjAcq(Size const& size, Acq* const acq, Plane3D<float> const& plane) :
        _size(size), _acq(acq), _plane(plane) {
    Cam const* cam = acq->getCam();

    // x,y z position of the camera in robot
    Point3D<float> v(cam->getMatC2R()(Rect(3, 0, 1, 3)));
    _distPlaneCam = _plane.distanceTo(v);

    Mat rot_cam1TOrob = cam->getMatC2R()(Rect(0, 0, 3, 3));
    Mat rot_robTOcam2 = _plane.getOneBasis() * (Mat_<float>(3, 3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);

    _rot_cam1TOcam2 = rot_robTOcam2 * rot_cam1TOrob;
    _rot_cam2TOcam1 = _rot_cam1TOcam2.t();

//    // Compute the size of the projected image
//    if (this->plane.a == 0 && this->plane.b == 0) { // The plan is normal at z axis
//    // Find the restrictive parameter about the size of the projected plane
//        float fSrc = cam->getFocal().height;
//        float elevation = cam->getRob2Cam().rx;
//        float aperAngle = cam->getAperAngle().height;
//        float var1 = fSrc * cos(aperAngle + elevation) / (cos(aperAngle) * cam->getSize().width);
//        float var2 = tan(elevation + aperAngle / 2) - tan(elevation + aperAngle / 2);
//        float f2_1 = size.width * var1;
//        float f2_2 = size.height / var2;
//
//        if (f2_1 < f2_2) {
//            this->focal = f2_1;
//            this->size.x = (int) round(f2_1 / var1) + 1;
//            this->size.y = (int) round(f2_1 * var2) + 1;
//        }
//        else {
//            this->focal = f2_2;
//            this->size.x = (int) round(f2_2 / var1) + 1;
//            this->size.y = (int) round(f2_2 * var2) + 1;
//        }
//    }
//    else {
//        std::cout << "Process of no horizontal plane not implemented yet" << std::endl;
//    }
}

Mat ProjAcq::getMat(eColorType ctype) {
    if (matMap.empty()) {
        eColorType ctype = BGR; // always convert matrix using RGB for consistency
        Mat mat = Mat_<Scalar_<uint8_t>>(getSize());

        // TODO compute projected matrix here

        matMap.insert(pair<eColorType, Mat>(ctype, mat));
    }

    return Image::getMat(ctype);
}

Mat ProjAcq::cam2plane(Mat pt_px) {
    Cam const* cam = _acq->getCam();

    // TODO pre-compute affine transformation coefficients

    if (pt_px.size[0] == 2) {
        pt_px.push_back(1.f);
    }
    Mat vec = _rot_cam1TOcam2 * cam->getMatI2C() * pt_px;

    vec *= (_distPlaneCam / vec.at<float>(2, 0)); // put the point on the plane

    vec = _rot_cam2TOcam1 * vec;
    vec.push_back(1.f);

    vec = cam->getMatC2R() * vec;
    vec.pop_back(1);

    return vec;
}

Mat ProjAcq::plane2cam(Mat pt_cm) {
    Cam const* cam = _acq->getCam();

    // TODO pre-compute affine transformation coefficients

    Mat vec = _plane.project(Pt3D(pt_cm)).toCv();

    vec.push_back(1.f);
    vec = cam->getMatR2C() * vec;
    vec.pop_back(1);

    vec /= vec.at<float>(2);

    vec = cam->getMatC2I() * vec;
    vec.pop_back(1);

    return vec;
}
