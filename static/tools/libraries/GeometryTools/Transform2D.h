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

template<typename T>
class Transform2D {
private:
    Transform2D() :
            mat(cv::Mat_<T>(3, 3)) {
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
            mat(cv::Mat_<T>(3, 3)) {

        // copy rotation part
        T crz = std::cos(rz);
        T srz = std::sin(rz);
        cv::Mat rotAB_Z = (cv::Mat_<T>(2, 2) <<
                crz, srz,
                -srz, crz);
        cv::Mat rotAB = mat(cv::Rect(0, 0, 2, 2));
        rotAB = rotAB_Z;

        // copy translation part
        cv::Mat trslAB_A = (cv::Mat_<T>(2, 1) << x, y);
        cv::Mat trslBA_B = -(rotAB * trslAB_A);
        trslBA_B.copyTo(mat(cv::Rect(2, 0, 1, 2)));

        // copy last row
        cv::Mat lastRow = (cv::Mat_<T>(1, 3) << T(0), T(0), T(1));
        lastRow.copyTo(mat(cv::Rect(0, 2, 3, 1)));
    }
    virtual ~Transform2D() {
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

        // copy last row
        cv::Mat lastRow = (cv::Mat_<T>(1, 3) << T(0), T(0), T(1));
        lastRow.copyTo(mat(cv::Rect(0, 2, 3, 1)));

        return trsfBA;
    }
};
#endif

#endif /* TRANSFORM3D_H_ */
