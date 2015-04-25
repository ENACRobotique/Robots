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

ProjAcq::ProjAcq(Size const& size, Acq* const acq, Plane3D<float> const& plane) :
        _size(size), _acq(acq), _plane(plane) {
    Cam const* cam = acq->getCam();

    Point3D<float> v(cam->getMatC2R()(Rect(3, 0, 1, 3)));  // x,y z position of the camera in robot
    _distPlaneCam = _plane.distanceTo(v);

    Mat rot_cam1TOrob = cam->getMatC2R()(Rect(0, 0, 3, 3));
    Mat rot_robTOcam2 = _plane.getOneBasis() * (Mat_<float>(3, 3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);

    _rot_cam1TOcam2 = rot_robTOcam2 * rot_cam1TOrob;
    _rot_cam2TOcam1 = _rot_cam1TOcam2.t();

    ////Compute the _size
        // Project the 4 vertices of the camera on the plane
            Point3D<float> ptTl_plane(_plane.interLine(cam->getOcPx_R(cam->getTopLeft()), cam->getTopLeft())); // // Top left Tl
            Point3D<float> ptTr_plane(_plane.interLine(cam->getOcPx_R(cam->getTopRight()), cam->getTopRight())); // Top right Tr
            Point3D<float> ptBl_plane(_plane.interLine(cam->getOcPx_R(cam->getBottomLeft()), cam->getBottomLeft())); // Bottom left Bl
            Point3D<float> ptBr_plane(_plane.interLine(cam->getOcPx_R(cam->getBottomRight()), cam->getBottomRight())); // Bottom right Br
            Point3D<float>  tabPtPlane[4] = {ptTl_plane, ptBl_plane, ptBr_plane, ptTr_plane};
        // Find a basis in the plane witch minimize the useless area (black pixels)
        // Current solution one of the side of the image is the longest side of the projected shape on the plane
        // Not working with shape having angles > 90°
            // Find the longest side
            Vector3D<float> tabSide[4];
            float side_lgest = 0.0;
            int idx_side_lgest;
            for(int i=0; i<4; i++){
                tabSide[i] = Vector3D<float> (tabPtPlane[i], tabPtPlane[(i+1)%4]);
                if(side_lgest < tabSide[i].norm()){
                    side_lgest = tabSide[i].norm();
                    idx_side_lgest = i;
                }
            }
            // Compute the ratio of the circumscribe rectangle
            float width, height;
            float idx_vertexFar;
            float lgestHeight = 0.0;
            for(int i=0; i<(4-2); i++){
                Point3D<float> pt_o = tabPtPlane[(idx_side_lgest)%4];
                Point3D<float> pt_e = tabPtPlane[(idx_side_lgest+1)%4];
                Vector3D<float> height(_plane.getClosestPtBtwnLineAndPt(pt_o, pt_e, tabPtPlane[i]),tabPtPlane[(idx_side_lgest + 2 + i)%4] );
                if(lgestHeight < height.norm()){
                    lgestHeight = height.norm();
                    idx_vertexFar = i;
                }
            }
            if(lgestHeight > side_lgest){
                width = lgestHeight;
                height = side_lgest;
            }
            else{
                width = side_lgest;
                height = lgestHeight;
            }
            float ratioIn = size.width/size.height;
            float rationProj = width/height;
            float ratioInProj_w = size.width/width;
            float rationInProj_h = size.height/height;
            float scale;
            if(rationProj > ratioIn){
                scale = size.width/width;
            }
            else{
                scale = size.height/height;
            }
//            _size.width = width*scale;
//            _size.height = height*scale;

            // TODO Compute the rotation of the new rectangle

}

Mat ProjAcq::getMat(eColorType ctype) {
    if (matMap.empty()) {
        eColorType ctype = BGR; // always convert matrix using RGB for consistency
        Mat mat = Mat_<Scalar_<uint8_t>>(getSize());

        // TODO compute projected matrix here

        matMap.insert(pair<eColorType, Mat>(ctype, mat));
    }

    return Image::getMat(ctype);
}

Mat ProjAcq::cam2plane(Mat pt_px) {
    Cam const* cam = _acq->getCam();

    // TODO pre-compute affine transformation coefficients

    if (pt_px.size[0] == 2) {
        pt_px.push_back(1.f);
    }
    Mat vec = _rot_cam1TOcam2 * cam->getMatI2C() * pt_px;

    vec *= (_distPlaneCam / vec.at<float>(2, 0)); // put the point on the plane

    vec = _rot_cam2TOcam1 * vec;
    vec.push_back(1.f);

    vec = cam->getMatC2R() * vec;
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
