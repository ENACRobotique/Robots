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

template<typename T>
class Transform3D {
private:
    Transform3D() :
            mat(cv::Mat_<T>(4, 4)) {
    }

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
            mat(cv::Mat_<T>(4, 4)) {
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
        cv::Mat rotAB = mat(cv::Rect(0, 0, 3, 3));
        rotAB = rotAB_Z * rotAB_Y * rotAB_X;

        cv::Mat trslAB_A = (cv::Mat_<T>(3, 1) << x, y, z);
        mat(cv::Rect(0, 3, 1, 3)) = -(rotAB * trslAB_A);
        mat(cv::Rect(3, 0, 4, 1)) = (cv::Mat_<T>(1, 4) << 0, 0, 0, 1);

        std::cout << "Built Transform: " << mat << std::endl;
    }
    virtual ~Transform3D() {
    }

    const cv::Mat& getMatrix() const {
        return mat;
    }

    Transform3D getReverse() const {
        Transform3D trsfBA;

        cv::Mat rotBA = trsfBA.mat(cv::Rect(0, 0, 3, 3));
        rotBA = mat(cv::Rect(0, 0, 3, 3)).t();

        cv::Mat trslBA_B = mat(cv::Rect(0, 3, 1, 3));
        trsfBA.mat(cv::Rect(0, 0, 3, 3)) = -(rotBA * trslBA_B);

        std::cout << "Built Reversed Transform: " << trsfBA.mat << std::endl;

        return trsfBA;
    }
};
#endif

#endif /* TRANSFORM3D_H_ */
