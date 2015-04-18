/*
 * Plane3D.h
 *
 *  Created on: 18 avr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_GEOMETRYTOOLS_PLANE3D_H_
#define LIB_GEOMETRYTOOLS_PLANE3D_H_

template<typename T>
class Plane3D {
public:
    T a, b, c, d;

    Plane3D(T a, T b, T c, T d) :
            a(a), b(b), c(c), d(d) {
    }
    virtual ~Plane3D() {
    }
};

#endif /* LIB_GEOMETRYTOOLS_PLANE3D_H_ */
