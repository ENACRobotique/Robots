#ifndef LIB_GEOMETRYTOOLS_POINT3D_H_
#define LIB_GEOMETRYTOOLS_POINT3D_H_

#include <Vector3D.h>
#include <iostream>

#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#endif

template<typename T>
class Point3D;

template<typename T>
std::ostream& operator<<(std::ostream& out, const Point3D<T>& p);

template<typename T>
class Point3D {
public:
    static const Point3D origin;

    T x, y, z;

    Point3D(T x, T y, T z) :
            x(x), y(y), z(z) {
    }
#ifdef USE_OPENCV
    Point3D(const cv::Mat& m) :
            x(m.at<T>(0)), y(m.at<T>(1)), z(m.at<T>(2)) {
    }
#endif
    virtual ~Point3D() {
    }

    Point3D operator+(const Vector3D<T>& v) const {
        return {x + v.x(), y + v.y(), z + v.z()};
    }
    Vector3D<T> operator-(const Point3D& p) const {
        return {x - p.x, y - p.y, z - p.z};
    }

    friend std::ostream& operator<< <T>(std::ostream& out, const Point3D& v);
};

template<typename T>
std::ostream& operator<<(std::ostream& out, const Point3D<T>& p) {
    out << "(" << p.x << ";" << p.y << ";" << p.z << ")";
	return out;
}

template<typename T>
const Point3D<T> Point3D<T>::origin {0, 0, 0};

#endif /* LIB_GEOMETRYTOOLS_POINT3D_H_ */
