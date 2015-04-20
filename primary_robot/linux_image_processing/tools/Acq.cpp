#include "Acq.h"



cv::Mat Acq::getMat(eColorType ctype) {
    matmap::iterator resIt = matMap.find(ctype);
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


Cam* Acq::getCam(){
    return cam;
}

ProjAcq Acq::projectOnPlane(Plane3D<float> plane, Vector2D<int> size){
    ProjAcq proAcq(size, plane);

    return proAcq;
}
