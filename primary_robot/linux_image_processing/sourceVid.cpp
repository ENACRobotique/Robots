#include "sourceVid.hpp"

#if USER == YOYO
// Path to the image sources for YOYO
string strFramePattern("/home/yoyo/Robots/primary_robot/linux_image_processing/Images/Table2015.png");
string strFrameRaw("/home/yoyo/Robots/primary_robot/linux_image_processing/Videos/Feux.mp4");
string strFrameHSV("/home/yoyo/Robots/primary_robot/linux_image_processing/Images/HSV.png");
#elif USER == LUDO
// Path to the image sources for Ludo
string strFramePattern("Images/Table2015.png");
string strFrameRaw("MyVideo.avi");
string strFrameHSV("Images/HSV.png");
#endif

int initCapture(string& title, VideoCapture& srcCap, int valBegVid, bool offsetVid){
	// Open the video file for reading
	switch(SRC_VID){ //
	case 0:
		srcCap.open(0);  // FIXME: check the argument
		break;
	case 1:
		srcCap.open(strFrameRaw.c_str());
		break;;
	}

    // if not success, exit program
	if ( !srcCap.isOpened() ){
		 cout << "Error: Cannot open the source video file" << endl;
		 return -1;
	}

    // Start the video at x ms
	if(offsetVid)
		srcCap.set(CV_CAP_PROP_POS_MSEC, valBegVid);

	namedWindow(title,CV_WINDOW_AUTOSIZE);

	return 0;
}
