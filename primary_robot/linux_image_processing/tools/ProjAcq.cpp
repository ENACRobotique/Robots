#include <Point2D.h>
#include <Point3D.h>
#include <tools/Acq.h>
#include <tools/Cam.h>
#include <tools/ProjAcq.h>
#include <cstdint>
#include <map>
#include <utility>

using namespace std;
using namespace cv;

//#define DBG_DISP

ProjAcq::ProjAcq(Size const& size, Acq* const acq, Plane3D<float> const& plane) :
        _size(size), _acq(acq), _plane(plane) {
    Cam const* cam = acq->getCam();

    Point3D<float> v(cam->getMatC2R()(Rect(3, 0, 1, 3)));
    _distPlaneCam = _plane.distanceTo(v);

    Mat rot_cam1TOrob = cam->getMatC2R()(Rect(0, 0, 3, 3));
    Mat rot_robTOcam2 = _plane.getOneBasis() * (Mat_<float>(3, 3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);

    _rot_cam1TOcam2 = rot_robTOcam2 * rot_cam1TOrob;
    _rot_cam2TOcam1 = _rot_cam1TOcam2.t();

    //// Create cam2 associated to the projected plane (z_cam2 is normal to the plane). Same origin than cam1.
    // /!\ For now works only for a x-axis rotation from the frame of the cam1
    // TODO to generalize
    Transform3D<float> tf = cam->getRob2Cam();
    float w1 = cam->getSize().width;
    float h1 = cam->getSize().height;
    float f1 = cam->getFocal().height;
    float av1_h = cam->getAperAngle().height/2;
    float theta = tf.getRx() - M_PIl;

#ifdef DBG_DISP
    cout <<"w1 = "<<w1<<", h1 = "<<h1<<", f1 = "<<f1<<endl;
    cout <<"ah1_h = "<<ah1_h/M_PIl*180.<<"°"<<", av1_h = "<<
            av1_h/M_PIl*180.<<"°"<<", theta = "<<theta/M_PIl*180.<<"°"<<endl;
#endif

    // Compute the dimensions of the new image from cam2
    float f2 = f1*cos(theta);

    float h2b = f2*tan(theta + av1_h);
    float h2s = f2*tan(theta - av1_h);
    float h2 = h2b - h2s;

    float d1 = sqrtf(pow(f1, 2) + pow(h1/2, 2));
    float d2 = sqrtf(pow(f2, 2) + pow(h2b, 2));
    float w2b = w1*d2/d1;

    cv::Size sz2;
    sz2.height = 2*h2b;
    sz2.width = w2b;

    _size.height = h2;
    _size.width = w2b;

#ifdef DBG_DISP
    cout <<"f1 = "<<f1<<", f2 = "<<f2<<endl;
    cout <<"h2b = "<<h2b<<", h2s = "<<h2s<<", h2 = "<<h2<<endl;
    cout <<"d1 = "<<d1<<", d2 = "<<d2<<", w2b = "<<w2b<<endl;
    cout<<"Size of the projected image = ("<<sz2.width<<", "<<sz2.height<<")p"<<endl;
    cout<<"Size of the useful projected image = ("<<_size.width<<", "<<_size.height<<")p"<<endl;
#endif

    // Get the position of the new cam and create it
    float x2 = cam->getRob2Cam().getReverse().getMatrix().at<float>(0,3),
          y2 = cam->getRob2Cam().getReverse().getMatrix().at<float>(1,3),
          z2 = cam->getRob2Cam().getReverse().getMatrix().at<float>(2,3);
    Transform3D<float> tf2(x2, y2, z2, 180.*M_PI/180., 0, 0);  // Cam2 to rob
    _CamProj = new Cam(f2, sz2, tf2);

    // Compute an array which contains the index of each pixel common to the two images (cam1 and camProj)
    Mat mat = Mat(_size, _acq->getMat(BGR).type());
    _idxImgProjInImg1 = Mat_<Point2i>(_size.width, _size.height);

    Mat K2_I2C = _CamProj->getMatI2C();
    Mat K1_C2I = this->_acq->getCam()->getMatC2I();
    for(int j=0; j<_size.height; j++){
        for(int i=0; i<_size.width; i++){
            Mat pt = (Mat_<float>(3,1)<< (float)i, (float)j, 1.);    // a point in imgProj
            pt = K2_I2C*pt; // point in camProj
            pt = this->_rot_cam2TOcam1*pt; // point in cam1
            pt = K1_C2I*(pt/pt.at<float>(2)); // point in image of cam1
            Point2i c(int(round(pt.at<float>(0))), int(round(pt.at<float>(1))));

            if( c.x > 0  &&  c.y > 0  &&  c.x < w1  &&  c.y < h1){
                _idxImgProjInImg1.at<Point2i>(i,j).x = c.y;
                _idxImgProjInImg1.at<Point2i>(i,j).y = c.x;
            }
            else{
                _idxImgProjInImg1.at<Point2i>(i,j) = Point2i(-1, -1);
            }
        }
    }
}

Mat ProjAcq::getMat(eColorType ctype) {
    if (matMap.empty()) {
        eColorType ctype = BGR; // always convert matrix using RGB for consistency
        Mat matProj = Mat(_size, _acq->getMat().type());
        cout<<"mat.size() = ("<<matProj.cols<<", "<<matProj.rows<<")"<<endl;

        // TODO compute projected matrix here
        Vec3b *px, *px0;
        for(int j=0; j<matProj.cols; j++){
            for(int i=0; i<matProj.rows; i++){
                Point2i* idx = _idxImgProjInImg1.ptr<Point2i>(j,i);
                px = matProj.ptr<Vec3b>(i,j);

                if(idx->x != -1  &&  idx->y != -1){
                    px0 = this->getAcq()->getMat(ctype).ptr<Vec3b>(idx->x, idx->y);
                    px->val[0] = px0->val[0];
                    px->val[1] = px0->val[1];
                    px->val[2] = px0->val[2];
                }
                else{
                    px->val[0] = 0;
                    px->val[1] = 0;
                    px->val[2] = 0;
                }
            }
        }

        matMap.insert(make_pair(ctype, matProj));
    }

    return Image::getMat(ctype);
}

Mat ProjAcq::cam2plane(Mat pt_px) {
    Cam const* cam = _acq->getCam();

    // TODO pre-compute affine transformation coefficients

    if (pt_px.size[0] == 2) {
        pt_px.push_back(1.f);
    }

    Mat vec = _rot_cam1TOcam2 * cam->getMatI2C() * pt_px; // pt_px from image of cam1 to cam2
    cout<<"pt_px_C2 = "<<vec<<endl;
    vec *= (_distPlaneCam / vec.at<float>(2, 0)); // put the point on the plane
    cout<<"pt_px_C2onPl = "<<vec<<endl;
    vec = _rot_cam2TOcam1 * vec;
    vec.push_back(1.f);
    cout<<"pt_px_C1Pl = "<<vec<<endl;
    vec = cam->getMatC2R() * vec;
    cout<<"pt_px_R = "<<vec<<endl;
    vec.pop_back(1);

    return vec;
}

Mat ProjAcq::plane2cam(Mat pt_cm) {
    Cam const* cam = _acq->getCam();

    // TODO pre-compute affine transformation coefficients

    Mat vec = _plane.project(Pt3D(pt_cm)).toCv();

    vec.push_back(1.f);
    vec = cam->getMatR2C() * vec;
    vec.pop_back(1);

    vec /= vec.at<float>(2);

    vec = cam->getMatC2I() * vec;
    vec.pop_back(1);

    return vec;
}

Mat ProjAcq::imProj2Plane(Mat pt_px){
    // TODO pre-compute affine transformation coefficients
    if (pt_px.size[0] == 2) {
        pt_px.push_back(1.f);
    }
    Mat vec = _CamProj->getMatI2C()*pt_px;
    vec *= (_distPlaneCam / vec.at<float>(2, 0)); // put the point on the plane
    vec.push_back(1.f);
    vec = _CamProj->getMatC2R() * vec;
    vec.pop_back(1);

    return vec;
}

Mat ProjAcq::imProj2PlaneAtDistFromCam(Mat pt_px, float d){
    // TODO pre-compute affine transformation coefficients
    if (pt_px.size[0] == 2) {
        pt_px.push_back(1.f);
    }
    Mat vec = _CamProj->getMatI2C()*pt_px;
    vec *= (d / vec.at<float>(2, 0)); // put the point on the plane
    vec.push_back(1.f);
    vec = _CamProj->getMatC2R() * vec;
    vec.pop_back(1);

    return vec;
}

float ProjAcq::getDistPlane2Cam(){
    return _distPlaneCam;
}

