#ifndef LIB_GEOMETRYTOOLS_VECTOR3D_H_
#define LIB_GEOMETRYTOOLS_VECTOR3D_H_

#include <Point3D.h>
#include <Vector2D.h>
#include <iostream>

#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#endif

template<typename T>
class Vector3D {
protected:
    T _x, _y, _z;
    bool _norm;

public:

    Vector3D() :
            _x(0), _y(0), _z(0), _norm(true) {
    }
    Vector3D(const Vector3D& v) :
            _x(v._x), _y(v._y), _z(v._z), _norm(false) {
    }
    Vector3D(const T _x, const T _y, const T _z) :
            _x(_x), _y(_y), _z(_z), _norm(false) {
    }
    Vector3D(const Point3D<T>& a, const Point3D<T>& b) :
            _x(b.x - a.x), _y(b.y - a.y), _z(b.z - a.z), _norm(false) {
    }
    Vector3D(const Vector2D<T> v, T z) :
            _x(v._x), _y(v._y), _z(z), _norm(false) {
    }
#ifdef USE_OPENCV
    Vector3D(const cv::Mat& m) :
            _x(m.at<T>(0)), _y(m.at<T>(1)), _z(m.at<T>(2)), _norm(false) {
    }
#endif
    ~Vector3D() {
    }

    const T& x() const {
        return _x;
    }
    const T& y() const {
        return _y;
    }
    const T& z() const {
        return _z;
    }

    T operator*(const Vector3D& v) const { // Dot product
        return _x * v._x + _y * v._y + _z * v._z;
    }
    Vector3D<T> operator^(const Vector3D& v) const { // Cross product
        return Vector3D((_y * v._z - _z * v._y), (-_x * v._z + _z * v._x), (_x * v._y - _y * v._x));
    }

    Vector3D operator+(const Vector3D& v) const {
        return {_x + v._x, _y + v._y, _z + v._z};
    }
    Vector3D operator-(const Vector3D& v) const {
        return {_x - v._x, _y - v._y, _z - v._z};
    }
    Vector3D operator*(const T& r) const {
        return {_x*r, _y*r, _z*r};
    }
    Vector3D operator/(const T& r) const {
        return {_x/r, _y/r, _z/r};
    }

    Vector3D& operator+=(const Vector3D& v) {
        return *this = *this + v;
    }
    Vector3D& operator-=(const Vector3D& v) {
        return *this = *this - v;
    }
    Vector3D& operator*=(const T& r) {
        return *this = *this * r;
    }
    Vector3D& operator/=(const T& r) {
        return *this = *this / r;
    }
    Vector3D& operator=(const Vector3D& v) {
        _x = v._x;
        _y = v._y;
        _z = v._z;
        return *this;
    }

    bool operator==(const Vector3D& v) const {
        return v._x == _x && v._y == _y && v._z == _z;
    }
    bool operator!=(const Vector3D& v) const {
        return !(*this == v);
    }

    T norm() const {
        return std::sqrt(_x * _x + _y * _y + _z * _z);
    }
    T normSq() const {
        return _x * _x + _y * _y + _z - _z;
    }
    Vector3D<T>& normalize() {
        if (!_norm && (_x || _y || _z)) {
            T n = this->norm();
            _x /= n;
            _y /= n;
            _z /= n;
        }
        _norm = true;
        return *this;
    }
//    void rotate(const T& theta) {  // TODO
//    }
    T angle(const Vector3D& v) {
        return std::acos(*this * v / std::sqrt(normSq() * v.normSq()));
    }

    bool isZero(T eps) {
        return (std::abs(_x) <= eps) && (std::abs(_y) <= eps) && (std::abs(_z) <= eps);
    }
};

template<typename T>
Vector3D<T> operator*(const T& n, const Vector3D<T>& v) {
    return v * n;
}

template<typename T>
std::ostream& operator<<(std::ostream& out, Vector3D<T>& v) {
    out << "(" << v.x << ";" << v.y << ";" << v.z << ")";
    return out;
}

#endif /* LIB_GEOMETRYTOOLS_VECTOR3D_H_ */
