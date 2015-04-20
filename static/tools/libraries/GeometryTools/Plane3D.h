/*
 * Plane3D.h
 *
 *  Created on: 18 avr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_GEOMETRYTOOLS_PLANE3D_H_
#define LIB_GEOMETRYTOOLS_PLANE3D_H_

#include "Point3D.h"

template<typename T>
class Point3D;

template<typename T>
class Plane3D {
public:
    // 3D plane (ax + by + cz + d = 0)
    T a, b, c, d;
    bool norm;

    Plane3D(T a, T b, T c, T d) :
            a(a), b(b), c(c), d(d), norm(false) {}
    virtual ~Plane3D() {}


    T distanceTo(const Point3D<T>& pt){
        T dist;
        // TODO

        return dist;
    }

    void normalize(){
        if(!norm){
            const T n = (T) sqrt(a*a + b*b + c*c);  // FIXME

            a /= n;
            b /= n;
            c /= n;
            d /= n;
        }
    }

};

#endif /* LIB_GEOMETRYTOOLS_PLANE3D_H_ */
