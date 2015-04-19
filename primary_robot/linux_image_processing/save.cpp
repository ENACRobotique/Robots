#include "save.h"



/**
 * \brief Initialize the writer, to save the capture
 * 		The new capture overwrite the previous capture
 * \param VideoCapture frameCap
 * \return int: 0 if initialization is OK, -1 otherwise
 *  TODO: take into account the case of a video or a capture of multiple frames
 */
int initSave(VideoCapture frameCap, VideoWriter& oVideoWriter){
	// Get the width  and height of frames of the video
	double dWidth = frameCap.get(CV_CAP_PROP_FRAME_WIDTH);
	double dHeight = frameCap.get(CV_CAP_PROP_FRAME_HEIGHT);
	Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
	cout << "Frame size : " << dWidth << " x " << dHeight << endl;

	// Initialize the VideoWriter object
	oVideoWriter.open("saveVideo.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true);
	//if not initialize the VideoWriter successfully, exit the program
	if ( !oVideoWriter.isOpened() ) {
		cout << "ERROR: Failed to write the video" << endl;
		return -1;
	}

	return 0;
}


/**
 * \brief
 * \param
 * \return
 * TODO: Add tests
 */
int save(VideoWriter& oVideoWriter, Mat& frame){
	oVideoWriter.write(frame);
	return 0;
}
