/*
 * Frame.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_ACQ_H_
#define TOOLS_ACQ_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Point2D.h>
#include <iostream>
#include <map>
#include <utility>

class Cam;

template<typename T> class Point2D;

enum ColorType {
    RGB,
    HSV
};

class Acq {
protected:
    using matmap = std::map<ColorType, cv::Mat>;

    matmap matMap;
    Cam* cam;

public:
    Acq(cv::Mat mat, ColorType ctype, Cam* cam):cam(cam) {
        matMap.insert(std::pair<ColorType, cv::Mat>(ctype, mat));
    }
    virtual ~Acq() {
    }

    cv::Mat getMat(ColorType ctype = RGB) {
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

                matMap.insert(std::pair<ColorType, cv::Mat>(ctype, ret));
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
};

#endif /* TOOLS_ACQ_H_ */
