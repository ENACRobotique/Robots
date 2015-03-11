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

class Vector2D {
    public:
        Vector2D();
        Vector2D(float _x, float _y);
        Vector2D(const Point2D& a, const Point2D& b);
        ~Vector2D();

        ERROR normVec(float& n)const;
        ERROR convPts2Vec(const Point2D& a, const Point2D& b);
        ERROR dotVecs(const Vector2D& v, float& d)const;
        ERROR crossVecs(const Vector2D& v, float& c)const;
        ERROR rotVec(const float& theta);

        float x;
        float y;
};

#endif /* LIB_GEOMETRYTOOLS_VECTOR2D_H_ */
