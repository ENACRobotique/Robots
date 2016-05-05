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
#include "bn_debug.h"

#include "tools.hpp"
#include "params.hpp"
#include "process.hpp"
#include "performance.hpp"
#include "save.hpp"
#include "sourceVid.hpp"

//#define CALIB_HSV
#define FAKE_RECV_MSG
#ifdef FAKE_RECV_MSG
// Warning Use only one of the next two defines
#define DEV_POS
//#define DEV_OBJ
#endif

// Pose of the robot in the reference fram of the table
#define X_T 218.
#define Y_T 166.
#define Z_T ((15. - 90.) * DEG2RAD)

#ifdef CALIB_HSV
// To remove
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
Scalar hsvT_min,hsvT_max;
/// Global Variables
const int hT_slider_max = 180, svT_slider_max = 256;
int hminT_slider=10, hmaxT_slider=21,
    sminT_slider=150, smaxT_slider=256,
    vminT_slider=50, vmaxT_slider=256;

void on_Ttrackbar(int, void*){
    hsvT_min = Scalar(hminT_slider, sminT_slider, vminT_slider);
    hsvT_max = Scalar(hmaxT_slider, smaxT_slider, vmaxT_slider);
}
#endif


//##### Main #####
int main(int argc, char* argv[]) {
    Perf& perf = Perf::getPerf();  //& ?

    cv::namedWindow("HSV");

#ifdef CALIB_HSV
    cv::namedWindow("RGL_HSV");
    cv::createTrackbar("H min", "RGL_HSV", &hminT_slider, hT_slider_max, on_Ttrackbar );
    cv::createTrackbar("H max", "RGL_HSV", &hmaxT_slider, hT_slider_max, on_Ttrackbar );
    cv::createTrackbar("S min", "RGL_HSV", &sminT_slider, svT_slider_max, on_Ttrackbar );
    cv::createTrackbar("S max", "RGL_HSV", &smaxT_slider, svT_slider_max, on_Ttrackbar );
    cv::createTrackbar("V min", "RGL_HSV", &vminT_slider, svT_slider_max, on_Ttrackbar );
    cv::createTrackbar("V max", "RGL_HSV", &vmaxT_slider, svT_slider_max, on_Ttrackbar );
    on_Ttrackbar(0,NULL);
#endif

    // Initialize cameras
    map<Cam*, VideoCapture*> camList;
    camList.insert(make_pair(
//            new Cam(0, 516.3, Size(640, 480), Transform3D<float>(0, 12.7, 26.7, 226. * M_PI / 180., 0, 0)),  // Position camera ?
//            new Cam(516.3, Size(640, 480), Transform3D<float>(0, 17, 31.5, 221. * M_PI / 180., 0, 0), 0),
            new Cam(516.3, Size(640, 480), Transform3D<float>(0, 8.5, 32, 221. * M_PI / 180., 0, 0), 0), // Robomovie
            //            new VideoCapture("MyVideo.avi")));
//            new VideoCapture(0)));
//            new VideoCapture("../2016/Captures/1cube.jpg"))); // "Robomovie"
//            new VideoCapture("../2016/Captures/cubesBiais.jpg"))); // "Robomovie"
//            new VideoCapture("../2016/Captures/cubes4x4x4.jpg"))); // "Robomovie"
//            new VideoCapture("../2016/Captures/violetShell.jpg"))); // "Robomovie"
//            new VideoCapture("../2016/Captures/cubesFace.jpg"))); // "Robomovie"
//            new VideoCapture("../2016/Captures/cylFar.jpg"))); // "Robomovie"
                new VideoCapture("../2016/Captures/dolphin_198_146_15.jpg"))); // "Robomovie, positioning"


    // Initialize processes
    std::map<eVidTypeProc, Process*> processMap;
    processMap[PROC_POS] = new ProcAbsPosNTree(camList.begin()->first, "../simu/testpoints.csv", PROC_POS);
//    processMap[PROC_POS] = new ProcIDObj(camList.begin()->first, "../2016/listObj.csv", PROC_OBJ);

    // Initialize botnet
    bn_init();

    Mat frameRaw;
    if (!camList.begin()->second->read(frameRaw)) { //if not success, break loop
        cout << "Cannot read the frame from source video file" << endl;
        return -1;
    }


    bool quit = false;
    bool succesAcq = false;
    int ret;
    sMsg inMsg;
    std::map<eVidTypeProc, Process*>::iterator itProc;
    vector<Acq*> acqList;

#ifdef FAKE_RECV_MSG
    // ************** For dev purpose: start user ***************
#ifdef DEV_POS
    // Received request message for positioning
    inMsg.header.type = E_VID_REQ;
    inMsg.payload.vidReq.type = PROC_POS;
    inMsg.payload.vidReq.pose.x = 198.;
    inMsg.payload.vidReq.pose.y = 146.;
    inMsg.payload.vidReq.pose.theta = 15.*DEG2RAD;
#endif
#ifdef DEV_OBJ
    // Received request message for objects recognition
    inMsg.header.type = E_VID_REQ;
    inMsg.payload.vidReq.type = PROC_OBJ;
    inMsg.payload.vidReq.pose.x = 198.;  // Warning: Update the position wrt to the image source.
    inMsg.payload.vidReq.pose.y = 146.;
    inMsg.payload.vidReq.pose.theta = 15.*DEG2RAD;
#endif
    // Simulate a reception
    ret = 1;
    //**********************************************************/
#endif

    do {
        // Communications
#ifdef FAKE_RECV_MSG
		if(ret){ // A message has been received
#else
        ;
        if(ret = bn_receive(&inMsg)){ // A message has been received
#endif
		    perf.beginFrame();

		    ret = 0; // 1 loop

		    // Set the right process
		    switch(inMsg.header.type){
		    case E_VID_REQ: // The message is for the video processing program
                switch (inMsg.payload.vidReq.type) {
                case PROC_POS: // The message is a request for video positioning
                    itProc = processMap.find(PROC_POS); // Get the right process
                    if(itProc == processMap.end()){
                        std::cout<<"Cannot find process PROC_POS\n";
                        break;
                    }

                    std::cout<<"main(): M4\n";
                    // Get the list of acquisition
                    std::cout<<"itProc->second->getCamList().size = "<<itProc->second->getCamList().size()<<std::endl;
                    for (Cam* c : itProc->second->getCamList()) {
                        map<Cam*, VideoCapture*>::iterator it = camList.find(c);

                        // Read a new frame from the video source
    //                    *(it->second) >> frameRaw; // Need to read 5 times to get the last frame
    //                    *(it->second) >> frameRaw;
    //                    *(it->second) >> frameRaw;
    //                    *(it->second) >> frameRaw;
//                        if (!it->second->read(frameRaw)) {  //if not success, break loop
//                            cout << "Cannot read the frame from source video file." << endl;
//                            continue;
//                        }
                        if (frameRaw.size() != c->getSize()) {
                            cout<< "skip cam c <- (frameRaw.size() = "<<frameRaw.size()<<") != (c->getSize() = "<<c->getSize()<<")\n";
                            continue;
                        }
                        acqList.push_back(new Acq(frameRaw, BGR, c));
                    }

                    // Check if the acquisition is a sucess
                    if(acqList.size() != 0){
                        succesAcq = true;
                    }
                    break;
                    case PROC_OBJ: // The message is a request for video recognition
                    itProc = processMap.find(PROC_OBJ); // Get the right process
                    if(itProc == processMap.end()){
                        std::cout<<"Cannot find process PROC_POBJ\n";
                        break;
                    }

                    std::cout<<"main(): M5\n";
                    // Get the list of acquisition
                    std::cout<<"itProc->second->getCamList().size = "<<itProc->second->getCamList().size()<<std::endl;
                    for (Cam* c : itProc->second->getCamList()) {
                        map<Cam*, VideoCapture*>::iterator it = camList.find(c);

                        // Read a new frame from the video source
    //                    *(it->second) >> frameRaw; // Need to read 5 times to get the last frame
    //                    *(it->second) >> frameRaw;
    //                    *(it->second) >> frameRaw;
    //                    *(it->second) >> frameRaw;
//                        if (!it->second->read(frameRaw)) {  //if not success, break loop
//                            cout << "Cannot read the frame from source video file." << endl;
//                            continue;
//                        }
                        if (frameRaw.size() != c->getSize()) {
                            cout<< "skip cam c <- (frameRaw.size() = "<<frameRaw.size()<<") != (c->getSize() = "<<c->getSize()<<")\n";
                            continue;
                        }
                        acqList.push_back(new Acq(frameRaw, BGR, c));
                    }

                    // Check if the acquisition is a sucess
                    if(acqList.size() != 0){
                        succesAcq = true;
                    }
                    break;
                    }
		        break;

                break;
            case E_DATA:
            case E_PING:
                break;
            default:
                bn_printfDbg("got unhandled msg: type%hhu sz%hhu", inMsg.header.type, inMsg.header.size);
                break;
            }

		    if(succesAcq){
                perf.endOfStep("acquisitions");

                // Execute the process
                std::cout<<"____ acqList.size() = "<<acqList.size()<<std::endl;
                itProc->second->process(acqList, AbsPos2D<float>(X_T, Y_T, Z_T), Uncertainty2D<float>(180, 180, 0, 10.f * M_PI / 180.f)); /// optim: 159.58, 21.58, 0
                std::cout<<"main(): M12\n";
                perf.endOfStep("process");
                cout<<"Mk2: process done\n";

    #ifdef CALIB_HSV
                // Test HSV
                cv::Mat im_hsv = acqList.front()->getMat(HSV);
                cv::Mat im_range;
                cv::inRange(im_hsv, hsvT_min, hsvT_max, im_range);
                cout<<"hsvT_min = "<<hsvT_min<<", hsvT_max = "<<hsvT_max<<endl;
    //            cv::imwrite("im_hsv.png", im_range);
                imshow( "RGL_HSV", im_range );
    #endif
                for (Acq* a : acqList)
                    delete a;



                imshow( "Display window", frameRaw );
    #ifndef CALIB_HSV
                waitKey(0);
    #endif
		    }
		}

#ifdef CALIB_HSV
            // Test HSV
            cv::Mat im_hsv = acqList.front()->getMat(HSV);
            cv::Mat im_range;
            cv::inRange(im_hsv, hsvT_min, hsvT_max, im_range);
            cout<<"hsvT_min = "<<hsvT_min<<", hsvT_max = "<<hsvT_max<<endl;
//            cv::imwrite("im_hsv.png", im_range);
            imshow( "RGL_HSV", im_range );
#endif

#ifdef CALIB_HSV
        switch(waitKey(1000./10)){
        case 0x110001B:
        case 27:
            quit = true;
            break;
        default:
            break;
        }
#endif
    } while (!quit);  // End while

    // Test
//    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", frameRaw);                   // Show our image inside it.
    waitKey(0);

    printf("End prog, bye bye!!!\n");

    return 0;
}

/*
 * TODO: DEBUG
 *
 *
 */
