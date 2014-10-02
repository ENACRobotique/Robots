/*
 * emitter.cpp
 *
 *  Created on: 2 oct. 2014
 *      Author: seb
 */

#include "yaml-cpp/yaml.h"

int main() {
    YAML::Emitter out;
    out << "Hello, World!";

    std::cout << "Here's the output YAML:\n" << out.c_str(); // prints "Hello, World!"
    return 0;
}
