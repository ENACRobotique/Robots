/*
 * vector.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_MATHTOOLS_VECTOR_H_
#define LIB_MATHTOOLS_VECTOR_H_

#include "error.h"

#include "point.h"

class Vector {
    public:
        Vector();
        Vector(float _x, float _y);
        Vector(const Point& a, const Point& b);
        ~Vector();

        ERROR normVec(float& n)const;
        ERROR convPts2Vec(const Point& a, const Point& b);
        ERROR dotVecs(const Vector& v, float& d)const;
        ERROR crossVecs(const Vector& v, float& c)const;
        ERROR rotVec(const float& theta);

        float x;
        float y;
};

#endif /* LIB_MATHTOOLS_VECTOR_H_ */
