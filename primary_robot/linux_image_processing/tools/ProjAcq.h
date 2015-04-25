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
#include <Point2D.h>
#include <Point3D.h>
#include <tools/Image.h>
#include <Vector2D.h>
#include <Vector3D.h>

class Acq;

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
    cv::Mat getMat(eColorType ctype = BGR) override;
    cv::Size getSize() override {
        return _size;
    }
    Plane3D<float> getPlane() {
        return _plane;
    }

    Pt2D cam2proj(Pt2D const& pt_pix);
    Pt2D proj2cam(Pt2D const& pt_pix);
    Pt3D proj2plane(Pt2D const& pt_pix);
    Vector2D<float> plane2proj(Pt3D const& pt_cm);
    Pt3D cam2plane(Pt2D const& pt_pix) {
        return Pt3D(cam2plane(pt_pix.toCv()));
    }
    template<typename T>
    cv::Point3_<float> cam2plane(cv::Point_<T> const& pt_pix) {
        cv::Mat p = cam2plane((cv::Mat_<float>(2, 1) << pt_pix.x, pt_pix.y));

        return cv::Point3_<float>(p.at<float>(0), p.at<float>(1), p.at<float>(2));
    }
    cv::Mat cam2plane(cv::Mat pt_px);
    Pt2D plane2cam(Pt3D const& pt_cm) {
        return Pt2D(plane2cam(pt_cm.toCv()));
    }
    cv::Mat plane2cam(cv::Mat pt_cm);
};

#endif /* TOOLS_PROJACQ_H_ */
