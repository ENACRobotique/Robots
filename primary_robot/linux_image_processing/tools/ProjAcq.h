/*
 * ProjAcq.h
 *
 *  Created on: 18 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_PROJACQ_H_
#define TOOLS_PROJACQ_H_

#include <opencv2/core/core.hpp>
#include <Plane3D.h>
#include <tools/Image.h>
#include <Vector2D.h>
#include <Vector3D.h>

class Acq;
class Cam;

class ProjAcq: public Image {
protected:
    using Pt3D = Point3D<float>;
    using Pt2D = Point2D<float>;

    cv::Size _size; // nb cols, nb rows
    Acq* const _acq;
    Plane3D<float> _plane;

    float _distPlaneCam;
    cv::Mat _rot_cam1TOcam2;
    cv::Mat _rot_cam2TOcam1;

public:
    ProjAcq(cv::Size const& size, Acq* const acq, Plane3D<float> const& plane);
    virtual ~ProjAcq() {
    }

    Acq* getAcq() const {
        return _acq;
    }
    cv::Mat getMat(eColorType ctype = RGB) override;
    cv::Size getSize() override {
        return _size;
    }
    Plane3D<float> getPlane() {
        return _plane;
    }

    Vector2D<float> cam2proj(Vector2D<float> const& pt_pix);
    Vector2D<float> proj2cam(Vector2D<float> const& pt_pix);
    Vector3D<float> proj2plane(Vector2D<float> const& pt_pix);
    Vector2D<float> plane2proj(Vector3D<float> const& pt_cm);
    Pt3D cam2plane(Pt2D const& pt_pix);
    Pt2D plane2cam(Pt3D const& pt_cm);
};

#endif /* TOOLS_PROJACQ_H_ */
