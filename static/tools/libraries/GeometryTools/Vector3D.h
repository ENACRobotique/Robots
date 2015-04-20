#ifndef LIB_GEOMETRYTOOLS_VECTOR3D_H_
#define LIB_GEOMETRYTOOLS_VECTOR3D_H_

#include <iostream>
#include <cmath>

template<typename T>
class Point3D;

template<typename T>
class Vector3D {
    public:
        Vector3D() : _x(0), _y(0), _z(0){}
        Vector3D(const Vector3D& v) : _x(v._x), _y(v._y), _z(v._z){}
        Vector3D(const T val) : _x(val), _y(val), _z(val){}
        Vector3D(const T _x, const T _y, const T _z) : _x(_x), _y(_y), _z(_z){}
        Vector3D(const Point3D<T>& a, const Point3D<T>& b) : _x(b.x - a.x), _y(b.y - a.y), _z(b.z - a.z){}
        Vector3D(const Vector2D<T> v, T z) : _x(v._x), _y(v._y), _z(z){}
        ~Vector3D(){}

        T operator*(const Vector3D& v) const{ // Dot product
            return _x*v._x + _y*v._y + _z*v._z;
        }
        Vector3D<T> operator^(const Vector3D& v) const{ // Cross product
            return Vector3D((_y*v._z - _z*v._y), (-_x*v._z + _z*v._x), (_x*v._y - _y*v._x));
        }

        Vector3D operator+(const Vector3D& v) const{
            return { _x + v._x, _y + v._y, _z + v._z };
        }
        Vector3D operator-(const Vector3D& v) const{
            return { _x - v._x, _y - v._y, _z - v._z };
        }
        Vector3D operator*(const T& r) const{
            return { _x*r, _y*r, _z*r };
        }
        Vector3D operator/(const T& r) const{
            return { _x/r, _y/r, _z/r };
        }

        Vector3D& operator+=(const Vector3D& v){
            return *this = *this + v;
        }
        Vector3D& operator-=(const Vector3D& v){
            return *this = *this - v;
        }
        Vector3D& operator*=(const T& r){
            return *this = *this * r;
        }
        Vector3D& operator/=(const T& r){
            return *this = *this / r;
        }
        Vector3D& operator=(const Vector3D& v){
            _x = v._x;
            _y = v._y;
            _z = v._z;
            return *this;
        }

        bool operator==(const Vector3D& v) const{
            return v._x == _x  &&  v._y == _y  && v._z == _z;
        }
        bool operator!=(const Vector3D& v) const{
            return !(*this == v);
        }

        T norm()const{
            return (T) sqrt(_x*_x + _y*_y + _z*_z);
        }
        T normSq()const{
            return _x*_x + _y*_y + _z-_z;
        }
        Vector3D<T> normalize(){
            if((_x == (T)0)  && (_y == (T)0) && (_z == (T)0) ){
                return this;
            }
            else{
                T n = this->norm();
                _x /= n;
                _y /= n;
                _z /= n;
                return this;
            }
        }
        void rotate(const T& theta){  // TODO
        }
        T angle(const Vector3D& v){
            return acos(*this * v / sqrt(normSq() * v.normSq()));
        }

        T dist2Point(Point3D<T> pt){  // FIXME
            Vector3D<T> vect(this - pt);

            return (T) vect.norm;
        }

        void setZero(){
            _x = _y = _z = (T)0;
        }
        bool isPositive(){
            if((_x >= (T)0)  &&  (_y >= (T)0)  &&  (_z >= (T)0) )
                return true;
            else
                return false;
        }
        bool isEmpty(){
            return ((_x == (T)0)  &&  (_y == (T)0)  &&  (_z == (T)0));
        }

        bool isZero(T eps){
            if((fabsf(_x) <= eps)  &&  (fabsf(_y) <= eps)  &&  (fabsf(_z) <= eps))
                return true;
             else
                 return false;
        }

        T _x;
        T _y;
        T _z;
};
