#include "ProjAcq.h"


ProjAcq::ProjAcq(Vector2D<int> _size, Cam cam, Plane3D<float> _plane){
    this->plane = _plane;


    // Compute the size of the projected image
    if(this->plane.a == 0  &&  this->plane.b ==0 ){ // The plan is normal at z axis
        // Find the restrictive parameter about the size of the projected plane
        Vector2D<int> sizeSrc;
        sizeSrc.x = cam.getSize().x;
        sizeSrc.y = cam.getSize().y;
        float fSrc = cam.getFocal();
        float elevation = cam.getConf().rx;
        float aperAngle = cam.getAperAngle();
        float var1 = fSrc*cos(aperAngle + elevation)/(cos(aperAngle)*sizeSrc.x);
        float var2 = tan(elevation+aperAngle/2) - tan(elevation+aperAngle/2);
        float f2_1 = _size.x*var1;
        float f2_2 = _size.y/var2;

        if(f2_1 < f2_2){
            this->focal = f2_1;
            this->size.x = (int) round(f2_1/var1)+1;
            this->size.y = (int) round(f2_1*var2)+1;
        }
        else{
            this->focal = f2_2;
            this->size.x = (int) round(f2_2/var1)+1;
            this->size.y = (int) round(f2_2*var2)+1;
        }
    }
    else{
        std::cout<<"Process of no horizontal plane not implemented yet"<<std::endl;
    }
}

cv::Mat ProjAcq::getMat(eColorType ctype) {
    matmapProj::iterator resIt = matMap.find(ctype);
    if (resIt == matMap.end()) {
        cv::Mat ret;
        int code = -1;

        // find a possible conversion
        for (resIt = matMap.begin(); code < 0 && resIt != matMap.end();
                resIt++) {
            switch (ctype) {
            case RGB:
                switch (resIt->first) {
                case HSV:
                    code = cv::COLOR_HSV2RGB;
                    break;
                default:
                    break;
                }
                break;
            case HSV:
                switch (resIt->first) {
                case RGB:
                    code = cv::COLOR_RGB2HSV;
                    break;
                default:
                    break;
                }
                break;
            default:
                break;
            }
        }

        if (code >= 0) {
            cv::cvtColor(resIt->second, ret, code);

            matMap.insert(std::pair<eColorType, cv::Mat>(ctype, ret));
        }
        else {
            std::cerr << "Can't convert to needed ctype!" << std::endl;
        }

        return ret;
    }
    else {
        return resIt->second;
    }
}

Vector2D<int> ProjAcq::getSize(){
    return this->size;
}
