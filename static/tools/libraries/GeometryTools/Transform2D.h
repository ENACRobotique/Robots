/*
 * Transform3D.h
 *
 *  Created on: 20 avr. 2015
 *      Author: lacoste_l
 */

#ifndef TRANSFORM2D_H_
#define TRANSFORM2D_H_

#ifdef USE_OPENCV
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <cmath>
#include <cassert>

template<typename T>
class Transform2D {
private:
    Transform2D() :
            mat(cv::Mat_<T>::eye(3, 3)) {
    }

protected:
    cv::Mat mat; // transform points expressed in A reference frame to B one

public:
    /**
     * Transformation from reference frame A to reference frame B
     * v_AB|A = (x, y)
     *          vector from center of reference frame A to center of B one, expressed in the A reference frame
     * r_AB = Rz
     *          each matrix rotates around the corresponding axis, example, Rx = (1 0 0 ; 0 cTheta sTheta ; 0 -sTheta cTheta)
     *          it allows for rx, ry, rz input angles to represent the natural "pose" of the B reference frame expressed in the A one
     *
     * pt|B = r_AB*(pt|A - v_AB|A)
     *
     *
     * matT_AB = r_AB     -r_AB*v_AB|A
     *           0  0            1
     *
     */
    Transform2D(const T x, const T y,
            const T rz) :
            mat(cv::Mat_<T>::eye(3, 3)) {

        // copy rotation part
        T crz = std::cos(rz);
        T srz = std::sin(rz);
        cv::Mat rotAB_Z = (cv::Mat_<T>(2, 2) <<
                crz, srz,
                -srz, crz);
        cv::Mat rotAB = rotAB_Z;
        rotAB.copyTo(mat(cv::Rect(0, 0, 2, 2)));

        // copy translation part
        cv::Mat trslAB_A = (cv::Mat_<T>(2, 1) << x, y);
        cv::Mat trslBA_B = -(rotAB * trslAB_A);
        trslBA_B.copyTo(mat(cv::Rect(2, 0, 1, 2)));
    }
    virtual ~Transform2D() {
    }

    template<typename T2>
    cv::Point_<T2> transformLinPos(cv::Point_<T2> const& in) const {
        cv::Mat p = transformLinPos((cv::Mat_<float>(2, 1) << in.x, in.y));

        return cv::Point_<T>(p.at<float>(0), p.at<float>(1));
    }

    template<typename T2>
    cv::Point_<T2> transformLinPos(cv::Point3_<T2> const& in) const {
        cv::Mat p = transformLinPos((cv::Mat_<float>(2, 1) << in.x, in.y));

        return cv::Point_<T>(p.at<float>(0), p.at<float>(1));
    }

    cv::Mat transformDir(cv::Mat in) const {
        cv::Mat R = mat(cv::Rect(0, 0, 2, 2));
        return R * in(cv::Rect(0, 0, 1, 2));
    }

    cv::Mat transformLinPos(cv::Mat in) const {
        if (in.size[0] == 2) {
            in.push_back(1.f);
        }
        else if (in.size[0] == 3) {
            in.at<float>(2) = 1;
        }
        else {
            assert(0);
        }
        cv::Mat ret = mat * in;
        ret.pop_back(1);
        return ret;
    }

    const cv::Mat& getMatrix() const {
        return mat;
    }

    Transform2D getReverse() const {
        Transform2D trsfBA;

        // copy transposed rotation part
        cv::Mat rotBA = mat(cv::Rect(0, 0, 2, 2)).t();
        rotBA.copyTo(trsfBA.mat(cv::Rect(0, 0, 2, 2)));

        // copy adapted translation part
        cv::Mat trslBA_B = mat(cv::Rect(2, 0, 1, 2));
        cv::Mat trslAB_A = -(rotBA * trslBA_B);
        trslAB_A.copyTo(trsfBA.mat(cv::Rect(2, 0, 1, 2)));

        return trsfBA;
    }
};
#endif

#endif /* TRANSFORM3D_H_ */
