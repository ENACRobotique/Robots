/*
 * DisplayImage.cpp
 *
 *  Created on: 23 févr. 2014
 *      Author: yoyo
 */

#include <opencv2/core/core.hpp>
#include <processes/ProcAbsPos.h>
#include <processes/Process.h>
#include <tools/Acq.h>
#include <tools/Position2D.h>
#include <tools/Uncertainty2D.h>
#include <iostream>
#include <vector>

int bn_init();
int printf(const char *__format);

using namespace cv;
using namespace std;

#include "shared/botNet_core.h"

#include "tools.hpp"
#include "params.hpp"
#include "process.hpp"
#include "performance.hpp"
#include "save.hpp"
#include "sourceVid.hpp"

//###############
//#### TODO ####
//##############

//####################
//#### Information ##
//###################
// To param cam manually:
// In terminal -> $ locate svv;  $ cd ->svv;  $ ./svv /dev/video1
//##################################
//############## Main ##############
//##################################
int main(int argc, char* argv[]) {
    Perf p;
    sPosOrien posOriRobot;
    Mat framePattern;
    Mat frameRaw;

    vector<Process*> processList;
    processList.push_back(new ProcAbsPos(""));

    bn_init();
    while (1) {
        p.beginFrame();

        for (Process* p : processList) {
            vector<Acq*> acqList;

            for (Cam* c : p->getCamList()) {
                // Read a new frame from the video source
                if (!c->cap.read(frameRaw)) {  //if not success, break loop
                    cout << "Cannot read the frame from source video file" << endl;
                    break;
                }

                acqList.push_back(new Acq());
            }

            p->process(acqList, Position2D<float>(), Uncertainty2D<float>());
        }

        p.endFrame();
    }  // End while

    printf("End loop\n");

    return 0;
}

