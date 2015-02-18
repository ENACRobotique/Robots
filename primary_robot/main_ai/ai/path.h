/*
 * path.h
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#ifndef AI_PATH_H_
#define AI_PATH_H_

#define MAX_RETRIES 1 //FIXME bug with simulation

#include "tools.h"
#include "math_types.h"

class Path {
    public:
        Path();
        ~Path();

        void sendRobot();
        void stopRobot();

        int sendSeg(const sPt_t *p, const sVec_t *v);


    private:
        sPath_t _path;
};


void followProg();

#endif /* AI_PATH_H_ */

