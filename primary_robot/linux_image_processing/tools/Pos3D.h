/*
 * Pos3D.h
 *
 *  Created on: 22 Dec 2015
 *      Author: Fulbok
 */

#ifndef TOOLS_POS3D_H_
#define TOOLS_POS3D_H_

#include <opencv2/core/core.hpp>
#include <Point3D.h>
//#include <tools/RelPos2D.h>
#include <Transform3D.h>
#include <cmath>
#include <iostream>

#ifdef USE_BOTNET
#include <messages-position.h>
#endif

//template<typename T>
//class AbsPos3D;

//template<typename T>
//std::ostream& operator<<(std::ostream& out, const Pos3D<T>& p);


template<typename T>
class Pos3D {
    Point3D<T> _p; // (cm x cm x cm)
    T _roll; // (rad)
    T _pitch; // (rad)
    T _yaw; // (rad)

public:
    Pos3D() :
            _roll(0), _pitch(0), _yaw(0) {
    }

    /**
     * x, y, z in centimeters
     * roll in radians
     * pitch in radians
     * yaw in radians
     */
    Pos3D(T x, T y, T z, T roll, T pitch, T yaw) :
            _p(x, y, z), _roll(roll), _pitch(pitch), _yaw(yaw) {
    }

    Pos3D(T x, T y, T z, T roll) :
            _p(x, y, z), _roll(roll), _pitch((T)0), _yaw((T)0) {
    }

    /**
     * p.x, p.y, p.z in centimeters
     * roll in radians
     * pitch in radians
     * yaw in radians
     */
    Pos3D(Point3D<T> p, T roll, T pitch, T yaw) :
            _p(p), _roll(roll), _pitch(pitch), _yaw(yaw) {
    }

    Pos3D(Vector3D<T> v, T roll, T pitch, T yaw) :
            _p(v), _roll(roll), _pitch(pitch), _yaw(yaw) {
    }

    Pos3D(cv::Mat p, T roll, T pitch, T yaw) :
            _p(p.at<T>(0), p.at<T>(1), p.at<T>(2)), _roll(roll), _pitch(pitch), _yaw(yaw) {
    }

#ifdef USE_BOTNET
    Pos3D(const s3DPos& p) :
            _p(p.x, p.y, p.z, p.roll, p.pitch, p.yaw) {
    }
#endif

    T const& x() const {
        return _p.x;
    }

    T const& y() const {
        return _p.y;
    }

    T const& z() const {
        return _p.z;
    }

    T const& roll() const {
        return roll;
    }

    T const& pitch() const {
        return pitch;
    }

    T const& yaw() const {
        return yaw;
    }


    Pos3D operator*(const T& r) const {
        return {Point3D<T>(_p.x*r, _p.y*r, _p.z*r), _roll*r, _pitch*r, _yaw*r};
    }
    Pos3D operator+(const Pos3D& v) const {
        return {Point3D<T>(_p.x + v._p.x, _p.y + v._p.y, _p.z + v._p.z)
            ,_roll + v._roll, _pitch + v._pitch, _yaw + v._yaw};
    }

    Pos3D& operator+=(const Pos3D& v) {
        return *this = *this + v;
    }

    Point3D<T> getPt3D(){
        return _p;
    }

    Vector3D<T> getRxyz(){
        return Vector3D<T>(_roll, _pitch, _yaw);
    }
};

template<typename T>
std::ostream& operator<<(std::ostream& out, const Pos3D<T>& p) {
    out << "(" << p.x() << " cm;" << p.y() << " cm;" << p.z() << " cm;"
        << p.roll() * T(180) / T(M_PI) << " deg;"
        << p.pitch() * T(180) / T(M_PI) << " deg;"
        << p.yaw() * T(180) / T(M_PI) << " deg)";

    return out;
}

#endif /* TOOLS_POS3D_H_ */
