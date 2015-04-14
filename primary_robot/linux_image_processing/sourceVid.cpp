#include "sourceVid.h"

int initCapture(VideoCapture& srcCap, int valBegVid, bool offsetVid){
	// Open the video file for reading
	switch(SRC_VID){ //
	case 0:
		srcCap.open(0);  // FIXME: check the argument
		break;
	case 1:
		srcCap.open("/home/yoyo/Robots/primary_robot/linux_image_processing/Videos/Feux.mp4");
		break;
	case 2:
		srcCap.open("/home/Ludo...");
		break;
	}

    // if not success, exit program
	if ( !srcCap.isOpened() ){
		 cout << "Error: Cannot open the source video file" << endl;
		 return -1;
	}

    // Start the video at x ms
	if(offsetVid)
		srcCap.set(CV_CAP_PROP_POS_MSEC, valBegVid);

	return 0;
}
