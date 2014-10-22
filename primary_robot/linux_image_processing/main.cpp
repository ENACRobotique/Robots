/*
 * DisplayImage.cpp
 *
 *  Created on: 23 févr. 2014
 *      Author: yoyo
 */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>



//######################
//#### Para de simu ####
//######################
bool SAVE = false;
bool RGL_HSV = false;


using namespace cv;
using namespace std;
using namespace gpu;


//###############
//#### Infos ####
//###############
    /*
    Feu rouge: HSVmin 170,160,60
               HSVmax 180,256,256 // Hmax=180
    Feu Jaune: HSVmin 22,160,60
               HSVmax 38,256,256
    */

    // Paramétrer cam: Ds terminal -> locate svv    cd /home/yoyo/Bureau/svv      ./svv /dev/video1


//####################################################
//############### Proto fonctions ####################
//####################################################
//void Window_RGL_HSV();


//#########################################
//############## Var globales #############
//########################################
// Var RGL_HSV
    /*int lowerH=0, lowerS=0, lowerV=0;
    int upperH=180,upperS=256, upperV=256;
    int init_H = 50, init_S = 50, init_V = 50;
	*/

//#########################################
//############## Main #####################
//#########################################
int main(int argc, char* argv[]){
	int nb_frame=0;
	struct timeval tv_1, tv_2;

	// open the video file for reading
    VideoCapture cap("/media/yoyo/BCACD6C6ACD679FA/Dropbox/Club Robot ENAC/Coupe 2014/Robot principal/Traitement Vidéo/Feux_Multi-cas_R-J.mp4");
    // Open video from cam no 1
//    VideoCapture cap(0);

    // if not success, exit program
    if ( !cap.isOpened() ){
         cout << "Cannot open the video file" << endl;
         return -1;
    }

    // Start the video at 300ms
    //cap.set(CV_CAP_PROP_POS_MSEC, 300);

    //create a windows
    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE);
    namedWindow("Video_thresh_R",CV_WINDOW_AUTOSIZE);
    namedWindow("Video_thresh_J",CV_WINDOW_AUTOSIZE);

    // Réglage HSV
    /*if(RGL_HSV){
    Window_RGL_HSV();
    }
    */

    // Get the frames per seconds of the video
    double fps = cap.get(CV_CAP_PROP_FPS);
    cout << "Frame per seconds : " << fps << endl;

    // Get the width  and height of frames of the video
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cout << "Frame size : " << dWidth << " x " << dHeight << endl;

    // Write video in file
    VideoWriter oVideoWriter;
    if(SAVE){
    	Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
		// Initialize the VideoWriter object
		oVideoWriter = VideoWriter ("/home/yoyo/Documents/depot-ENAC-Robotique/primary_robot/linux_image_processing/MyVideo.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true);
		//if not initialize the VideoWriter successfully, exit the program
		if ( !oVideoWriter.isOpened() ) {
			cout << "ERROR: Failed to write the video" << endl;
			return -1;
		}
    }



    while(1){
    	gettimeofday(&tv_1, NULL);

        Mat img_brut, img_topview = Mat(781, 538*2, CV_8UC3);

        // Read a new frame from video
        bool bSuccess = cap.read(img_brut);
        //if not success, break loop
        if (!bSuccess){
			cout << "Cannot read the frame from video file" << endl;
			break;
		}

        // Writer the frame into the file
        //if(SAVE){
        	oVideoWriter.write(img_brut);
        //}

        // Redressement image
        	int j,i,u,v;
        	for(j=0;j<781;j++){
        	    for(i=0;i<538*2;i++){
        	        u = int((3195.*(j+1) - 5213.*sqrt(2)*(i+1) + 2804594.*sqrt(2) - 3961800.)/(10.*((j+1) - 1240.))-1.);
        	        v = int(-(1409.*(j+1) - 805.)/(5.*(j+1) - 6200.)-1.);

        	        if(u>=0 && u<640 && v>=0 && v<480){ // Accès au pixel
        	            for(int a=0;a<3 ; a++){ //sur les trois canaux
        	            	img_topview.at<Vec3b>(j,i)[a] = img_brut.at<Vec3b>(v,u)[a];
        	            }
        	        }
        	        else{
        	            for(int a=0;a<3 ; a++){ //sur les trois canaux
        	            	img_topview.at<Vec3b>(j,i)[a] = 0;
        	            }
        	        }
        	    }
        	}


        // Traitment thresold
			// Smooth color image
//        	Mat img_smooth = img_topview.clone();
//			for ( int i = 1; i <= 10; i = i + 2 ){
//				bilateralFilter ( img_topview, img_smooth, i, i*2, i/2 );
//			}
//        	imshow("lissée", img_smooth);

        Mat img_HSV, img_thres_R, img_thres_J;// img_thres_tri
        cvtColor(img_topview, img_HSV, CV_RGB2HSV); //RGB to HSV


        // Erosion operation
		Mat element = getStructuringElement( MORPH_ELLIPSE,
											 Size( 5, 5 ),
											 Point( 0, 0 ) );
		Mat element_dilate = getStructuringElement( MORPH_ELLIPSE,
													 Size( 8, 8 ),
													 Point( 0, 0 ) );
		/// For red
        	inRange(img_HSV , cv::Scalar(105,70,45), cv::Scalar(180,256,256), img_thres_R);

//        	pyrDown(img_HSV, img_HSV, Size((img_HSV.cols+1) / 2, (img_HSV.rows+1) / 2)); // Size(img_HSV.cols / 2, img_HSV.rows / 2)
//        	pyrUp(img_HSV, img_HSV, Size(img_HSV.cols * 2, img_HSV.rows * 2)); // img_HSV.size()
//
        	erode( img_thres_R, img_thres_R, element );
			dilate( img_thres_R, img_thres_R, element_dilate );

	        // Amélioration des contours
	       // Sobel(img_thres_R, img_thres_R, -1, 1, 1, 7, 1, 0, BORDER_DEFAULT );

//			Mat img_Canny;
//			Canny(img_thres_R, img_Canny, 0, 50, 5);
//
//			vector<Vec2f> lines;
//			HoughLines(img_Canny, lines, 1., CV_PI/180., 60);
//
//			for( size_t i = 0; i < lines.size(); i++ )
//			{
//			  float rho = lines[i][0], theta = lines[i][1];
//			  Point pt1, pt2;
//			  double a = cos(theta), b = sin(theta);
//			  double x0 = a*rho, y0 = b*rho;
//			  pt1.x = cvRound(x0 + 1000*(-b));
//			  pt1.y = cvRound(y0 + 1000*(a));
//			  pt2.x = cvRound(x0 - 1000*(-b));
//			  pt2.y = cvRound(y0 - 1000*(a));
//			  line(img_topview, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
//			}

			// Find contours
			std::vector<std::vector<cv::Point> > contours;
			Mat contourOutput = img_thres_R.clone();

			findContours( contourOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );//CV_CHAIN_APPROX_SIMPLE, CV_CHAIN_APPROX_NONE

/*
			//Draw the contours
				vector< vector<Point> > triangles;
				// Array for storing the approximation curve
				cv::Mat contourImage(img_topview.size(), CV_8UC3, cv::Scalar(0,0,0));
				cv::Scalar colors;
				colors = cv::Scalar(0, 0, 255);
				vector<Point>approxTriangle;
				// Test each contour
				for (size_t idx = 0; idx < contours.size(); idx++) {
					// approximate contour with accuracy proportional to the contour perimeter
					approxPolyDP(Mat(contours[idx]), approxTriangle, arcLength(cv::Mat(contours[idx]), true) * 0.075, true);

					// Cas du triangle
					// Skip small or non-convex objects
					//printf("\n\tfabs(contourArea(approxTriangle)) = %f: \n",fabs(contourArea(approxTriangle)));
					if ((approxTriangle.size() >= 3 ) && fabs(contourArea(approxTriangle)) > 10000 && isContourConvex(approxTriangle)){
//						drawContours(img_topview, contours,idx, colors, 2);
						// Adds elements to the bottom of the matrix
						triangles.push_back(approxTriangle);
					}
		            vector<Point>::iterator vertex;
		            for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
		                circle(img_topview, *vertex, 6, colors, 3);
		            }
				}
				// Draw triangles
				for (size_t i=0; i<triangles.size(); i++){
					const Point* point = &triangles[i][0];
					int points = (int) triangles[i].size();
					printf("\n\tpoints[%lu]=%d",i,points);
					if(i==(5-1)){
						printf("\nFin\n\n");
					}
					polylines(img_topview, &point, &points, 1, true, colors, 3, CV_AA);
				}

		/// For yellow
	        inRange(img_HSV , cv::Scalar(60,20,45), cv::Scalar(100,256,256), img_thres_J);
			erode( img_thres_J, img_thres_J, element );
			dilate( img_thres_J, img_thres_J, element_dilate );

			imshow("Video_thresh_J",img_thres_J);

			// Find contours
			std::vector<std::vector<cv::Point> > contoursJ;
			Mat contourOutputJ = img_thres_J.clone();
			findContours( contourOutputJ, contoursJ, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );//CV_CHAIN_APPROX_SIMPLE, CV_CHAIN_APPROX_NONE

			//Draw the contours
			Mat contourImageJ(img_topview.size(), CV_8UC3, cv::Scalar(0,0,0));
			Scalar colorsJ;
			colorsJ = cv::Scalar(0, 255, 255);
			for (size_t idx = 0; idx < contoursJ.size(); idx++) {
				cv::drawContours(img_topview, contoursJ, idx, colorsJ,4);
			}

*/
        // Show the frame in "MyVideo" window
        imshow("MyVideo", img_brut);
		imshow("brute redressee", img_topview);
        imshow("Video_thresh_R",img_thres_R);
 //       imshow("Video_thresh_J",img_thres_J);
//        imshow("img_Canny",img_Canny);

        //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
        if(waitKey(30) == 27){
			cout << "esc key is pressed by user" << endl;
			break;
        }

        gettimeofday(&tv_2, NULL);

        {
        	int delta_us = (tv_2.tv_sec - tv_1.tv_sec)*1000000 + tv_2.tv_usec - tv_1.tv_usec;
        	printf("%.2fFPS\n", 1000000./float(delta_us));
        }

        printf("frame %i\n", nb_frame++);
    }// End while

    return 0;
}


//###########################################
//############# Fonctions ###################
//###########################################
/*
void Window_RGL_HSV(){
    if(RGL_HSV){
        cvNamedWindow("Réglage HSV");
        // Change H, S, V
        createTrackbar("Hue", "Réglage HSV", &init_H, 256, MyCallbackForHSV,);
        createTrackbar("Saturation", "Réglage HSV", &init_S, 256, MyCallbackForHSV);
        createTrackbar("Value", "Réglage HSV", &init_V, 256, MyCallbackForHSV);

        cvCreateTrackbar("LowerV", "Réglage HSV", &lowerV, 256, NULL);
        cvCreateTrackbar("UpperV", "Réglage HSV", &upperV, 256, NULL);
    }
}
*/

//##########################################
/*
void MyCallbackForHSV(int iValueForH, void *userData)
{
	inRange(img_HSV , cv::Scalar(iValueForH,70,45), cv::Scalar(150,256,256), img_thres);
}
*/

//##########################################


//##########################################



