#ifndef LIB_GEOMETRYTOOLS_VECTOR3D_H_
#define LIB_GEOMETRYTOOLS_VECTOR3D_H_

#include <Vector2D.h>
#include <iostream>

#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#endif

#define RAD2DEG(x) (180.0/(x * M_PI))
#define DEG2RAD(x) (1/RAD2DEG(x))

template<typename T>
class Vector3D;

template<typename T>
std::ostream& operator<<(std::ostream& out, const Vector3D<T>& v);

template<typename T>
class Vector3D {
protected:
    T _x, _y, _z;
    bool _norm;

    Vector3D(const T _x, const T _y, const T _z, const bool _norm) :
            _x(_x), _y(_y), _z(_z), _norm(_norm) {
    }

public:
    static const Vector3D zero;
    static const Vector3D xAxis;
    static const Vector3D yAxis;
    static const Vector3D zAxis;

    Vector3D() :
            _x(0), _y(0), _z(0), _norm(true) {
    }
    Vector3D(const Vector3D& v) :
            _x(v._x), _y(v._y), _z(v._z), _norm(false) {
    }
    Vector3D(const T _x, const T _y, const T _z) :
            _x(_x), _y(_y), _z(_z), _norm(false) {
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
    T operator[](int idx){
        switch(idx){
        case 0: return _x; break;
        case 1: return _y; break;
        case 2: return _z; break;
        default: std::cout<<"type Vector3D<T> has no member "<<idx<<std::endl;
            return -1;  // FIXME: Is it relevant?
        }
    }
    Vector3D<T> operator^(const Vector3D& v) const { // Cross product
        return Vector3D((_y * v._z - _z * v._y), (-_x * v._z + _z * v._x), (_x * v._y - _y * v._x), _norm && v._norm);
    }

    Vector3D operator+(const Vector3D& v) const {
        return {_x + v._x, _y + v._y, _z + v._z, false};
    }
    Vector3D operator-(const Vector3D& v) const {
        return {_x - v._x, _y - v._y, _z - v._z, false};
    }
    Vector3D operator*(const T& r) const {
        return {_x*r, _y*r, _z*r, false};
    }
    Vector3D operator/(const T& r) const {
        return {_x/r, _y/r, _z/r, false};
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
        _norm = v._norm;
        return *this;
    }

    bool operator==(const Vector3D& v) const {
        return v._x == _x && v._y == _y && v._z == _z;
    }
    bool operator!=(const Vector3D& v) const {
        return !(*this == v);
    }

    T norm() const {
        return _norm ? T(1) : std::sqrt(_x * _x + _y * _y + _z * _z);
    }
    T normSq() const {
        return _norm ? T(1) : _x * _x + _y * _y + _z - _z;
    }
    Vector3D<T>& normalize() {
        if (!_norm && (_x || _y || _z)) {
            T n = norm();
            _x /= n;
            _y /= n;
            _z /= n;
        }
        _norm = true;
        return *this;
    }
    void rotate( Vector3D<T>& axis, const T& angle, int unit = 0) {  // 0==rad, 1 =deg
        axis.normalize();

        if(unit)
            angle = RAD2DEG(angle);

        T A[3][3];
        for(int row=0; row<3; row++)
            for(int col=0; col<3; col++)
                A[row][col] = (1-cos(angle))*axis[row]*axis[col];

        T B[3][3];
        B[0][0] = B[1][1] = B[2][2] = 0;
        B[2][1] = sin(angle)*axis[2];
        B[1][2] = -B[2][1];
        B[1][3] = sin(angle)*axis[1];
        B[3][1] = -B[1][3];
        B[3][2] = sin(angle)*axis[0];
        B[2][3] = -B[3][2];

        T C[3][3];
        C[0][1] = C[1][0] = C[0][2] = C[2][0] = C[1][2] = C[2][1] = (T)0;
        C[0][0] = C[1][1] = C[2][2] = cos(angle);

        T M[3][3];
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
                M[i][j] = A[i][j] + B[i][j] + C[i][j];

        Vector3D temp(this);
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
                this[j] = M[i][j]*temp[j];
    }
    void rotate(T Rx, T Ry, T Rz, int unit=0) // 0=rad, 1=deg
    {
        if(unit){
            Rx = DEG2RAD(Rx);
            Ry = DEG2RAD(Ry);
            Rz = DEG2RAD(Rz);
        }
        T a = cos(Rx);
        T b = sin(Rx);
        T c = cos(Ry);
        T d = sin(Ry);
        T e = cos(Rz);
        T f = sin(Rz);
        T nx = c*e*_x - c*f*_y + d*_z;
        T ny = (a*f + b*d*e)*_x + (a*e - b*d*f)*_y - b*c*_z;
        T nz = (b*f - a*d*e)*_x + (a*d*f + b*e)*_y + a*c*_z;
        this->_x = nx;
        this->_y = ny;
        this->_z = nz;

    }
    T angle(const Vector3D& v) {
        return std::acos(*this * v / std::sqrt(normSq() * v.normSq()));
    }

    bool isZero(T eps) {
        return (std::abs(_x) <= eps) && (std::abs(_y) <= eps) && (std::abs(_z) <= eps);
    }

#ifdef USE_OPENCV
    cv::Mat toCv() {
        return (cv::Mat_<T>(3, 1) << _x, _y, _z);
    }
#endif

    friend std::ostream& operator<<<T>(std::ostream& out, const Vector3D& v);
};

template<typename T>
Vector3D<T> operator*(const T& n, const Vector3D<T>& v) {
    return v * n;
}

template<typename T>
std::ostream& operator<<(std::ostream& out, const Vector3D<T>& v) {
    out << "(" << v._x << ";" << v._y << ";" << v._z << ")";
    return out;
}

template<typename T>
const Vector3D<T> Vector3D<T>::zero { 0, 0, 0, true };
template<typename T>
const Vector3D<T> Vector3D<T>::xAxis { 1, 0, 0, true };
template<typename T>
const Vector3D<T> Vector3D<T>::yAxis { 0, 1, 0, true };
template<typename T>
const Vector3D<T> Vector3D<T>::zAxis { 0, 0, 1, true };

#endif /* LIB_GEOMETRYTOOLS_VECTOR3D_H_ */
