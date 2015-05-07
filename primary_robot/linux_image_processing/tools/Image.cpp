/*
 * Image.cpp
 *
 *  Created on: 20 avr. 2015
 *      Author: ludo6431
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tools/Image.h>
#include <iostream>
#include <utility>

cv::Mat Image::getMat(eColorType ctype) {
    map::iterator resIt = matMap.find(ctype);
    if (resIt == matMap.end()) {
        cv::Mat ret;
        int code = -1;

        // find a possible conversion
        for (resIt = matMap.begin(); code < 0 && resIt != matMap.end();
                resIt++) {
            switch (ctype) {
            case BGR:
                switch (resIt->first) {
                case HSV:
                    code = cv::COLOR_HSV2BGR;
                    break;
                default:
                    break;
                }
                break;
            case HSV:
                switch (resIt->first) {
                case BGR:
                    code = cv::COLOR_BGR2HSV;
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
            cv::cvtColor(prev(resIt)->second, ret, code);

            matMap.insert(std::make_pair(ctype, ret));
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

cv::Size Image::getSize() {
    return matMap.begin()->second.size();
}
