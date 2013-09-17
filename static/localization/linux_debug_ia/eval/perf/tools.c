#include <stdlib.h>

#include "tools.h"

float uni_rand(float min, float max) {
    return min + ((max-min)*(float)rand())/(float)RAND_MAX;
}

