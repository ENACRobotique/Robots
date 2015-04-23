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
#include <tools/Cam.h>
#include <tools/Position2D.h>
#include <tools/Uncertainty2D.h>
#include <iostream>
#include <vector>

// For communications
//#include "../../network_config/messages.h"
//#include "../../network_config/messages-elements.h"
//#include "node_cfg.h"
//#include "../../static/communication/botNet/shared/botNet_core.h"
//#include "../../static/communication/botNet/shared/message_header.h"
//#include "../../network_config/roles.h"  // FIXME: ??? Is it relevant ???

using namespace cv;
using namespace std;

#include "shared/botNet_core.h"

#include "tools.hpp"
#include "params.hpp"
#include "process.hpp"
#include "performance.hpp"
#include "save.hpp"
#include "sourceVid.hpp"

//##### TODO ####


//##### Information ##
// To param cam manually:
// In terminal -> $ locate svv;  $ cd ->svv;  $ ./svv /dev/video1


//##### Main #####
int main(int argc, char* argv[]) {
    Perf perf;
    Mat frameRaw;

    map<Cam*, VideoCapture*> camList;
    camList.insert(std::pair<Cam*, VideoCapture*>(
            new Cam(521.3, Size(640, 480), Transform3D<float>(0, 107, 267, 225.*M_PI/180., 0, 0)),
//            new VideoCapture("MyVideo.avi")));
            new VideoCapture(0)));

    vector<Process*> processList;
    processList.push_back(new ProcAbsPos(camList.begin()->first, "simu/testpoints.csv"));

    bn_init();

    bool quit = false;
	do {
//		ret = bn_receive(&inMsg);
//
//		switch(inMsg.header.type){
//            case E_POS_CAM:
        perf.beginFrame();

        for (Process* p : processList) {
            vector<Acq*> acqList;

            for (Cam* c : p->getCamList()) {
                map<Cam*, VideoCapture*>::iterator it = camList.find(c);

                // Read a new frame from the video source
                if (!it->second->read(frameRaw)) {  //if not success, break loop
                    cout << "Cannot read the frame from source video file" << endl;
                    continue;
                }

                if(frameRaw.size() != c->getSize()){
                    continue;
                }

                imshow("rgb", frameRaw);

                acqList.push_back(new Acq(frameRaw, RGB, c));
            }

            perf.endOfStep("acquisitions");

            p->process(acqList, Position2D<float>(0, 0, 0), Uncertainty2D<float>(0, 0, 0, 0));

            for(Acq* a : acqList){
                delete a;
            }

            perf.endOfStep("process");
        }

        switch(waitKey(1000./10)){
        case 0x110001B:
        case 27:
            quit = true;
            break;
        default:
            break;
        }

        perf.endFrame();
//
//                break;
//            case E_DATA:
//            case E_PING:
//                break;
//            default:
//                bn_printfDbg("got unhandled msg: type%hhu sz%hhu", inMsg.header.type, inMsg.header.size);
//                break;
//		}  // End switch

	}while(!quit);  // End while

	printf("End loop\n");

	return 0;
}

