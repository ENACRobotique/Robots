/*
 * emitter.cpp
 *
 *  Created on: 2 oct. 2014
 *      Author: seb
 */

#include <yaml-cpp/emitter.h>
#include <iostream>

using namespace YAML;

struct Position {
    float x; // (cm)
    float y; // (cm)
    float theta; // (°)
};

Emitter& operator <<(Emitter& out, const Position& v) {
    out << BeginSeq << v.x << Comment("(cm)") << v.y << Comment("(cm)")
            << v.theta << Comment("(°)") << EndSeq;
    return out;
}

int main() {
    Position v { 1, 2, 3 };

    Emitter out;
    out << BeginSeq;
    {
        out << "Hello, World!";
        {
            out << BeginMap << Key << "blah" << Value << v << EndMap;
        }
    }
    out << EndSeq;

    std::cout << "Here's the output YAML:\n" << out.c_str(); // prints "Hello, World!"
    return 0;
}
