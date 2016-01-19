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
    T _x, _y, _z;

public:
    static const Point3D origin;

    Point3D() :
            _x(0), _y(0), _z(0) {
    }
    Point3D(T x, T y, T z) :
            _x(x), _y(y), _z(z) {
    }
    Point3D(Vector3D<T> v) :_x(v.x()), _y(v.y()), _z(v.z()){
    }

    #ifdef USE_OPENCV
    Point3D(const cv::Mat& m) :
            _x(m.at<T>(0)), _y(m.at<T>(1)), _z(m.at<T>(2)) {
    }
#endif
    virtual ~Point3D() {
    }

    T& x() {
        return _x;
    }
    T& y() {
        return _y;
    }
    T& z() {
        return _z;
    }

    T const& x() const {
        return _x;
    }
    T const& y() const {
        return _y;
    }
    T const& z() const {
        return _z;
    }

    Point3D operator+(const Vector3D<T>& v) const {
        return {_x + v.x(), _y + v.y(), _z + v.z()};
    }
    Vector3D<T> operator-(const Point3D& p) const {
        return {_x - p._x, _y - p._y, _z - p._z};
    }
    T operator[](const int i) const{
        if(i == 0) return _x;
        if(i == 1) return _y;
        if(i == 2) return _z;
        else{
            std::cout<<"Point3D: argument "<< i << " for operator [] is not in [0,2]\n";
            return (T)10e6;
        }
    }

#ifdef USE_OPENCV
    cv::Mat toCv() const {
        return (cv::Mat_<T>(3, 1) << _x, _y, _z);
    }
#endif

    friend std::ostream& operator<<<T>(std::ostream& out, const Point3D& v);
};

template<typename T>
std::ostream& operator<<(std::ostream& out, const Point3D<T>& p) {
    out << "(" << p.x() << ";" << p.y() << ";" << p.z() << ")";
    return out;
}

template<typename T>
const Point3D<T> Point3D<T>::origin { 0, 0, 0 };

#endif /* LIB_GEOMETRYTOOLS_POINT3D_H_ */
