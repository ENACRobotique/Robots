/*
 * ProjAcq.h
 *
 *  Created on: 18 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_PROJACQ_H_
#define TOOLS_PROJACQ_H_

#include "Acq.h"
#include "Cam.h"
#include "commonTypes.h"
#include <Vector2D.h>
#include <Plane3D.h>

template<typename T> class Plane3D;
template<typename T> class Vector2D;

class Acq;



class ProjAcq {
protected:
    using matmapProj = std::map<eColorType, cv::Mat>;

    matmapProj matMap;
    Vector2D<int> size; // nb cols, nb rows
    double focal;
    Plane3D<float> plane;

public:
    ProjAcq(Vector2D<int> _size, Cam _cam, Plane3D<float> _plane);

    Vector2D<int> getSize();
    Acq* getAcq();
    cv::Mat getMat(eColorType ctype = RGB);
    Vector2D<int> cam2proj(Vector2D<int> pt_pix);
    Vector2D<int> proj2cam(Vector2D<int> pt_pix);
    Vector2D<float> proj2plane(Vector2D<int> pt_pix);
    Vector2D<int> plane2proj(Vector2D<float> pt_cm);
    Vector2D<float> cam2plane(Vector2D<int> pt_pix);
    Vector2D<int> plane2cam(Vector2D<float> pt_cm);
    Plane3D<float> getPlane();
};

#endif /* TOOLS_PROJACQ_H_ */
