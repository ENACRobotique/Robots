/*
 * polygon2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sebastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_POLYGON2D_H_
#define LIB_GEOMETRYTOOLS_POLYGON2D_H_

template<typename T>
class Point2D;

template<typename T>
class Polygon2D {
    public:
        Polygon2D();
        ~Polygon2D();

        void checkPtInPolygon(const Point2D<T> pt, bool& ret) const{
            /*  std::array <Line, this->p.size()> l; //FIXME

              for (unsigned int i = 0; i < this->p.size(); i++) {
                  bool norm = false;
                  l[i].convPts2Line(this->p[i], this->p[(i + 1) % this->p.size()], norm);
              }

              int sg_prev = SIGN(l[0].a * pt.x + l[0].b * pt.y + l[0].c), sg = 0;

              for (unsigned int i = 1 ; i < this->p.size(); i++) {
                  sg = SIGN(l[i].a * pt.x + l[i].b * pt.y + l[i].c);
                  if (sg != sg_prev) {
                      ret = false;
                      return 0;
                  }
              }

              ret = true;
          */
        }

        std::vector <Point2D<T>> p;
};

#endif /* LIB_GEOMETRYTOOLS_POLYGON2D_H_ */
