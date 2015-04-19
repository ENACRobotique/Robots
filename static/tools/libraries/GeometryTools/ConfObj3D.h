#include <iostream>
#include <cmath>


template<typename T>
class ConfObj3D{
// TODO: Add methods, operators, constructors
public:
    ConfObj3D(): x(0), y(0), z(0), rx(0), ry(0), rz(0){}
    ConfObj3D(const T _x, const T _y, const T _z, const T _rx,
            const T _ry, const T _rz) :
            x(_x), y(_y), z(_z), rx(_rx), ry(_ry), rz(_rz){}
    ~ConfObj3D(){}

    T x;
    T y;
    T z;
    T rx;
    T ry;
    T rz;


};
