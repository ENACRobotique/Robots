/*
 * DisplayImage.cpp
 *
 *  Created on: 23 févr. 2014
 *      Author: yoyo
 */

#include <opencv2/core/core.hpp>
#include <processes/ProcAbsPos/ProcAbsPosNTree.h>
#include <processes/ProcAbsPos/ProcAbsPos.h>
#include <processes/ProcIDObj/ProcIDObj.h>
#include <processes/Process.h>
#include <tools/AbsPos2D.h>
#include <tools/Acq.h>
#include <tools/Cam.h>
#include <tools/Uncertainty2D.h>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

#include "shared/botNet_core.h"

#include "tools.hpp"
#include "params.hpp"
#include "process.hpp"
#include "performance.hpp"
#include "save.hpp"
#include "sourceVid.hpp"

//##### Main #####
int main(int argc, char* argv[]) {
    Perf& perf = Perf::getPerf();  //& ?
    Mat frameRaw;

    // Initialize cameras
    map<Cam*, VideoCapture*> camList;
    camList.insert(make_pair(
            new Cam(516.3, Size(640, 480), Transform3D<float>(0, 12.7, 26.7, 226. * M_PI / 180., 0, 0)),  // Position camera ?
            //            new VideoCapture("MyVideo.avi")));
//            new VideoCapture(0)));
            new VideoCapture("../Images/captures/z1.png"))); // "Robomovie"


    // Initialize processes
    vector<Process*> processList;
    processList.push_back(new ProcAbsPosNTree(camList.begin()->first, "../simu/testpoints.csv"));
    processList.push_back(new ProcAbsPosNTree(camList.begin()->first, "../playgroundObj/listObj.csv"));

    // Initialize botnet
    bn_init();

    if (!camList.begin()->second->read(frameRaw)) { //if not success, break loop
        cout << "Cannot read the frame from source video file" << endl;
        return -1;
    }


    bool quit = false;
    do {
        // Communications
//		ret = bn_receive(&inMsg);
//
//		switch(inMsg.header.type){
//            case E_POS_CAM:

        // Perform processes
        perf.beginFrame();
        for (Process* p : processList) {
            vector<Acq*> acqList;

            for (Cam* c : p->getCamList()) {
                map<Cam*, VideoCapture*>::iterator it = camList.find(c);

                // Read a new frame from the video source
//                if (!it->second->read(frameRaw)) {  //if not success, break loop
//                    cout << "Cannot read the frame from source video file" << endl;
//                    continue;
//                }

                if (frameRaw.size() != c->getSize()) {
                    cout<< "skip cam c <- (frameRaw.size() = "<<frameRaw.size()<<") != (c->getSize() = "<<c->getSize()<<")\n";
                    continue;
                }

//                imshow("rgb", frameRaw);

                acqList.push_back(new Acq(frameRaw, BGR, c));
            }

            perf.endOfStep("acquisitions");

            p->process(acqList, AbsPos2D<float>(145, 30, 5 * M_PI / 180.), Uncertainty2D<float>(180, 180, 0, 10.f * M_PI / 180.f)); /// optim: 159.58, 21.58, 0

            cout<<"Mk2: process done\n";
            for (Acq* a : acqList) {
                delete a;
            }

            perf.endOfStep("process");
        }

//        switch(waitKey(1000./10)){
//        case 0x110001B:
//        case 27:
//            quit = true;
//            break;
//        default:
//            break;
//        }

        perf.endFrame();

        quit = 1;
//
//                break;
//            case E_DATA:
//            case E_PING:
//                break;
//            default:
//                bn_printfDbg("got unhandled msg: type%hhu sz%hhu", inMsg.header.type, inMsg.header.size);
//                break;
//		}  // End switch

    } while (!quit);  // End while

    // Test
    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", frameRaw );                   // Show our image inside it.
    waitKey(0);

    printf("End prog\n");

    return 0;
}

/*
 * TODO: DEBUG
 *
 *
 */
