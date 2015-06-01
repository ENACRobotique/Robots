/*
 * View.cpp
 *
 *  Created on: 13 nov. 2014
 *      Author: ludo6431
 */

#include <gui/parts/View.h>

std::ostream& operator<<(std::ostream& os, const View& v) {
    os << "View of type: \"" << v._vid << "\"";
    return os;
}

