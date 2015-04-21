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

    Point3D<float> v(cam->getMatC2R()(Rect(0, 3, 1, 3)));
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
        eColorType ctype = RGB; // always convert matrix using RGB for consistency
        Mat mat = Mat_<Scalar_<uint8_t>>(getSize());

        // TODO compute projected matrix here

        matMap.insert(pair<eColorType, Mat>(ctype, mat));
    }

    return Image::getMat(ctype);
}

Point3D<float> ProjAcq::cam2plane(Pt2D const& pt_pix) {
    Cam const* cam = _acq->getCam();

    Mat px = (Mat_<float>(3, 1) << pt_pix.x, pt_pix.y, 1);
    Mat vec = _rot_cam1TOcam2 * cam->getMatI2C() * px;

    vec *= (_distPlaneCam / vec.at<float>(2, 0)); // put the point on the plane

    vec = _rot_cam2TOcam1 * vec;
    vec.push_back(1);

    Mat vec_rob = cam->getMatC2R() * vec;

    return Pt3D(vec_rob);
}

Point2D<float> ProjAcq::plane2cam(Pt3D const& pt_cm) {
    Cam const* cam = _acq->getCam();

    Mat vec = _plane.project(pt_cm).toCv();

    vec.push_back(float(1));
    vec = cam->getMatR2C() * vec;
    vec.pop_back(1);

    vec /= vec.at<float>(2);

    return Pt2D(cam->getMatC2I() * vec);
}
