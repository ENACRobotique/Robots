#ifndef LIB_GEOMETRYTOOLS_POINT3D_H_
#define LIB_GEOMETRYTOOLS_POINT3D_H_

#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#endif

template<typename T>

class Point3D {
public:
    T x, y, z;

    Point3D(T x, T y, T z) :
            x(x), y(y), z(z) {
    }
#ifdef USE_OPENCV
    Point3D(const cv::Mat& m) :
            x(m.at<T>(0)), y(m.at<T>(1)), z(m.at<T>(2)) {
    }
#endif
    virtual ~Point3D() {
    }
};

#endif /* LIB_GEOMETRYTOOLS_POINT3D_H_ */
