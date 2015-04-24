/*
 * Image.h
 *
 *  Created on: 20 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_IMAGE_H_
#define TOOLS_IMAGE_H_

#include <opencv2/core/core.hpp>
#include <Vector2D.h>
#include <map>

enum eColorType {
    BGR,
    HSV
};

class Image {
protected:
    using map = std::map<eColorType, cv::Mat>;
    map matMap;

public:
    virtual ~Image() {
    }

    virtual cv::Mat getMat(eColorType ctype = BGR);
    virtual cv::Size getSize();
};

#endif /* TOOLS_IMAGE_H_ */
