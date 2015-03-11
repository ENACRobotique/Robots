/*
 * Vector2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_VECTOR2D_H_
#define LIB_GEOMETRYTOOLS_VECTOR2D_H_

#include "error.h"
#include "Point2D.h"

template<typename T>
class Vector2D {
    public:
        Vector2D();
        Vector2D(T _x, T _y);
        Vector2D(const Point2D<T>& a, const Point2D<T>& b);
        ~Vector2D();

        void normVec(T& n)const;
        void convPts2Vec(const Point2D<T>& a, const Point2D<T>& b);
        void dotVecs(const Vector2D& v, T& d)const;
        void crossVecs(const Vector2D& v, T& c)const;
        void rotVec(const T& theta);

        T x;
        T y;
};

#endif /* LIB_GEOMETRYTOOLS_VECTOR2D_H_ */
