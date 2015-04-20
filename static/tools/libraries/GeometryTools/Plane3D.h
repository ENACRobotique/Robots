/*
 * Plane3D.h
 *
 *  Created on: 18 avr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_GEOMETRYTOOLS_PLANE3D_H_
#define LIB_GEOMETRYTOOLS_PLANE3D_H_

#include <Point3D.h>
#include <Vector3D.h>

#ifdef USE_OPENCV
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#endif

template<typename T>
class Plane3D {
protected:
    // 3D plane (ax + by + cz + d = 0)
    T _a, _b, _c, _d;
    bool norm;

public:
    Plane3D(T a, T b, T c, T d) :
            _a(a), _b(b), _c(c), _d(d), norm(false) {
    }
    Plane3D(const Vector3D<T>& p, const Vector3D<T>& n) :
            norm(false) {
        _a = n.x;
        _b = n.y;
        _c = n.z;
        _d = -(p * n);
    }
    virtual ~Plane3D() {
    }

    const T& a() const {
        return _a;
    }
    const T& b() const {
        return _b;
    }
    const T& c() const {
        return _c;
    }
    const T& d() const {
        return _d;
    }

    T distanceTo(const Point3D<T>& pt) {
        normalize();
        return _a * pt.x + _b * pt.y + _c * pt.z + _d;
    }

    Plane3D& normalize() {
        if (!norm) {
            Vector3D<T> nv { _a, _b, _c };
            T n = nv.norm();
            _a /= n;
            _b /= n;
            _c /= n;
            _d /= n;
            norm = true;
        }
        return *this;
    }

    /**
     * Normal vector
     * (not-normalized)
     */
    Vector3D<T> getZ() const {
        return {_a, _b, _c};
    }

    /**
     * A vector orthogonal to z
     * (not-normalized)
     */
    Vector3D<T> getOneX() const {
        int minI = 0;
        T minV = std::abs(_a);

        T absB = std::abs(_b);
        if (absB <= minV) {
            minI = 1;
            minV = absB;
        }

        if (std::abs(_c) <= minV) {
            minI = 2;
        }

        switch (minI) {
        default:
            case 0:
            return {0, -_c, _b};
        case 1:
            return {_c, 0, -_a};
        case 2:
            return {-_b, _a, 0};
        }
    }

    /**
     * A vector orthogonal to x (returned by getOneX()) and z
     * (not-normalized)
     */
    Vector3D<T> getOneY() const {
        return getZ() ^ getOneX();
    }

#ifdef USE_OPENCV
    cv::Mat getOneBasis() {
        normalize(); // ensures getZ returns a normalized vector

        Vector3D<T> x = getOneX().normalize();
        Vector3D<T> y = getOneY().normalize();
        Vector3D<T> z = getZ();

        return cv::Mat_<T>(3, 3) <<
                x.x, y.x, z.x,
                x.y, y.y, z.y,
                x.z, y.z, z.z;
    }
#endif
};

#endif /* LIB_GEOMETRYTOOLS_PLANE3D_H_ */