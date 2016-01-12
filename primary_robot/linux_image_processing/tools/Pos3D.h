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

template<typename T>
class AbsPos3D;

template<typename T>
std::ostream& operator<<(std::ostream& out, const Pos3D<T>& p);

template<typename T>
class Pos3D {
    Point3D<T> _p; // (cm x cm x cm)
    T _rx; // (rad)
    T _ry; // (rad)
    T _rz; // (rad)
    Vector3D<T> _camDir;

public:
    Pos3D() :
            _rx(0), _ry(0), _rz(0) {
    }

    /**
     * x, y, z in centimeters
     * rx in radians
     * ry in radians
     * rz in radians
     */
    Pos3D(T x, T y, T z, T rx, T ry, T rz) :
            _p(x, y, z), _rx(rx), _ry(ry), _rz(rz) {
    }

//    /**
//     * x, y, z in centimeters
//     * rx in radians
//     */
//    AbsPos2D(T x, T y, T theta, Vector2D<T> const& camDir) :
//            _p(x, y), _theta(theta), _camDir(camDir) {
//    }

//    /**
//     * p.x, p.y in centimeters
//     * theta in radians
//     */
//    AbsPos2D(Point2D<T> p, T theta, Vector2D<T> const& camDir) :
//            _p(p), _theta(theta), _camDir(camDir) {
//    }

    /**
     * p.x, p.y, p.z in centimeters
     * rx in radians
     * ry in radians
     * rz in radians
     */
    Pos3D(Point3D<T> p, T rx, T ry, T rz) :
            _p(p), _rx(rx), _ry(ry), _rz(rz) {
    }

    Pos3D(cv::Mat p, T rx, T ry, T rz) :
            _p(p.at<T>(0), p.at<T>(1), p.at<T>(2)), _rx(rx), _ry(ry), _rz(rz) {
    }

    // TODO : Implement botnet messages for 3D points
//#ifdef USE_BOTNET
//    Pos2D(const s2DPosAtt& p) :
//            _p(p.x, p.y), _theta(p.theta) {
//    }
//#endif

    T const& x() const {
        return _p.x;
    }

    T const& y() const {
        return _p.y;
    }

    T const& z() const {
        return _p.z;
    }

    T const& rx() const {
        return rx;
    }

    T const& ry() const {
        return ry;
    }

    T const& rz() const {
        return rz;
    }


    Pos3D operator*(const T& r) const {
        return {Point3D<T>(_p.x*r, _p.y*r, _p.z*r), _rx*r, _ry*r, _rz*r, _camDir};
    }
    Pos3D operator+(const Pos3D& v) const {
        return {Point3D<T>(_p.x + v._p.x, _p.y + v._p.y, _p.z + v._p.z)
            ,_rx + v._rx, _ry + v._ry, _rz + v._rz, v._camDir};

    }

    Pos3D& operator+=(const AbsPos2D& v) {
        return *this = *this + v;
    }


//    AbsPos2D operator+(const RelPos2D<T>& v) const {
////        Vector2D<T> const* camDir = &_camDir;
////        T nSq = camDir->normSq();
////        if(!nSq) {
////            camDir = &v.camDir();
////            nSq = camDir->normSq();
////            std::cout << "using camDir " << *camDir << " from vector..." << std::endl;
////        }
////
////        T dtheta = 0;
////        if(nSq) {
////            Vector2D<T> vv_rob(getTransform().transformDir(v.v().toCv())); // from pg2rob
////
////            dtheta = (vv_rob ^ *camDir) / nSq;
////        }
//
//        return {_p.x + v.x(), _p.y + v.y(), _theta + v.theta() /*+ dtheta*/, _camDir};
//    }
//    AbsPos2D operator-(const RelPos2D<T>& v) const {
//        return {_p.x - v.x(), _p.y - v.y(), _theta - v.theta(), _camDir};
//    }
//    RelPos2D<T> operator-(const AbsPos2D& p) const {
//        return {_p.x - p._p.x, _p.y - p._p.y, _theta - p._theta, _camDir};
//    }

//    Transform3D<T> getTransform() const {
//        return Transform3D<T>(_p.x, _p.y, _p.z, _rx(rx), _ry(ry), _rz(rz));
//    }

//    friend std::ostream& operator<<<T>(std::ostream& out, const Pos3D& p);
};

template<typename T>
std::ostream& operator<<(std::ostream& out, const Pos3D<T>& p) {
    out << "(" << p.x() << " cm;" << p.y() << " cm;" << p.z() << " cm;"
        << p.rx() * T(180) / T(M_PI) << " deg;"
        << p.ry() * T(180) / T(M_PI) << " deg;"
        << p.rz() * T(180) / T(M_PI) << " deg)";

    return out;
}

#endif /* TOOLS_POS3D_H_ */
