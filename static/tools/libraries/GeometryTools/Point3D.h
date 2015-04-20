
#ifndef LIB_GEOMETRYTOOLS_POINT3D_H_
#define LIB_GEOMETRYTOOLS_POINT3D_H_


template<typename T>

class Point3D {
public:
    T _x, _y, _z;

    Point3D(T x, T y, T z) :
            _x(x), _y(y), _z(z) {}
    virtual ~Point3D() {}
};

#endif /* LIB_GEOMETRYTOOLS_POINT3D_H_ */
