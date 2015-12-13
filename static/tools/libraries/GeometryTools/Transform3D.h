/*
 * Transform3D.h
 *
 *  Created on: 20 avr. 2015
 *      Author: lacoste_l
 */

#ifndef TRANSFORM3D_H_
#define TRANSFORM3D_H_

#ifdef USE_OPENCV
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <cmath>
#include <cassert>
#include <Transform2D.h>

template<typename T>
class Transform3D {
private:
    Transform3D() :
            mat(cv::Mat_<T>::eye(4, 4)) {
    }

    T _rx, _ry, _rz;

protected:
    cv::Mat mat; // transform points expressed in A reference frame to B one

public:
    /**
     * Transformation from reference frame A to reference frame B
     * v_AB|A = (x, y, z)
     *          vector from center of reference frame A to center of B one, expressed in the A reference frame
     * r_AB = Rz*Ry*Rx
     *          each matrix rotates around the corresponding axis, example, Rx = (1 0 0 ; 0 cTheta sTheta ; 0 -sTheta cTheta)
     *          it allows for rx, ry, rz input angles to represent the natural "pose" of the B reference frame expressed in the A one
     *
     * pt|B = r_AB*(pt|A - v_AB|A)
     *
     *
     * matT_AB = r_AB     -r_AB*v_AB|A
     *           0 0 0           1
     *
     */
    Transform3D(const T x, const T y, const T z,
            const T rx, const T ry, const T rz) :
            mat(cv::Mat_<T>::eye(4, 4)) {

        _rx = rx, _ry = ry, _rz = rz;

        // copy rotation part
        T crx = std::cos(rx);
        T srx = std::sin(rx);
        T cry = std::cos(ry);
        T sry = std::sin(ry);
        T crz = std::cos(rz);
        T srz = std::sin(rz);
        cv::Mat rotAB_X = (cv::Mat_<T>(3, 3) <<
                1, 0, 0,
                0, crx, srx,
                0, -srx, crx);
        cv::Mat rotAB_Y = (cv::Mat_<T>(3, 3) <<
                cry, 0, -sry,
                0, 1, 0,
                sry, 0, cry);
        cv::Mat rotAB_Z = (cv::Mat_<T>(3, 3) <<
                crz, srz, 0,
                -srz, crz, 0,
                0, 0, 1);
        cv::Mat rotAB = rotAB_Z * rotAB_Y * rotAB_X;
        rotAB.copyTo(mat(cv::Rect(0, 0, 3, 3)));

        // copy translation part
        cv::Mat trslAB_A = (cv::Mat_<T>(3, 1) << x, y, z);
        cv::Mat trslBA_B = -(rotAB * trslAB_A);
        trslBA_B.copyTo(mat(cv::Rect(3, 0, 1, 3)));
    }

    Transform3D(Transform2D<T> const& tr) :
            mat(cv::Mat_<T>::eye(4, 4)) {
        cv::Mat m33 = tr.getMatrix();

        // copy rotation part
        cv::Mat rotAB = m33(cv::Rect(0, 0, 2, 2));
        rotAB.copyTo(mat(cv::Rect(0, 0, 2, 2)));

        // copy translation part
        cv::Mat trslBA_B = m33(cv::Rect(2, 0, 1, 2));
        trslBA_B.copyTo(mat(cv::Rect(3, 0, 1, 2)));
    }

    virtual ~Transform3D() {
    }

    cv::Mat transformLinPos(cv::Mat in) const {
        if (in.size[0] == 3) {
            in.push_back(1.f);
        }
        else if (in.size[0] == 4) {
            in.at<float>(3) = 1;
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

    Transform3D getReverse() const {
        Transform3D trsfBA;

        // copy transposed rotation part
        cv::Mat rotBA = mat(cv::Rect(0, 0, 3, 3)).t();
        rotBA.copyTo(trsfBA.mat(cv::Rect(0, 0, 3, 3)));

        // copy adapted translation part
        cv::Mat trslBA_B = mat(cv::Rect(3, 0, 1, 3));
        cv::Mat trslAB_A = -(rotBA * trslBA_B);
        trslAB_A.copyTo(trsfBA.mat(cv::Rect(3, 0, 1, 3)));

        // copy last row
        cv::Mat lastRow = (cv::Mat_<T>(1, 4) << T(0), T(0), T(0), T(1));
        lastRow.copyTo(mat(cv::Rect(0, 3, 4, 1)));

        return trsfBA;
    }

    T getRx(){
        return _rx;
    }

    T getRy(){
        return _ry;
    }

    T getRz(){
        return _rz;
    }
};
#endif

#endif /* TRANSFORM3D_H_ */
