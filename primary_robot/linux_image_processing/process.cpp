/*
 * process.cpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

#include "params.hpp"
#include "process.hpp"

Scalar hsv_min,hsv_max;
/// Global Variables
const int h_slider_max = 180, sv_slider_max = 256;
int hmin_slider=110, hmax_slider=180, smin_slider=105, smax_slider=220, vmin_slider=100, vmax_slider=250;

void on_trackbar(int, void*){
	hsv_min = Scalar(hmin_slider, smin_slider, vmin_slider);
	hsv_max = Scalar(hmax_slider, smax_slider, vmax_slider);
}


int nb_fx_frame = 0;
float x_R = 2600;
float y_R = 1400;
float z_R = 0;
float angl_R = 0;

float Pass_R_T11 = cos(angl_R) / ( pow(cos(angl_R),2) + pow(sin(angl_R),2) ),
	  Pass_R_T12 = -sin(angl_R) / ( pow(cos(angl_R),2) + pow(sin(angl_R),2) ),
	  Pass_R_T13 = 0,
	  Pass_R_T14 = x_R,
	  Pass_R_T21 = sin(angl_R) / ( pow(cos(angl_R),2) + pow(sin(angl_R),2) ),
	  Pass_R_T22 = cos(angl_R) / ( pow(cos(angl_R),2) + pow(sin(angl_R),2) ),
	  Pass_R_T23 = 0,
	  Pass_R_T24 = y_R,
	  Pass_R_T31 = 0,
	  Pass_R_T32 = 0,
	  Pass_R_T33 = 1,
	  Pass_R_T34 = z_R,
	  Pass_R_T41 = 0,
	  Pass_R_T42 = 0,
	  Pass_R_T43 = 0,
	  Pass_R_T44 = 1;

Mat Pass_R_T = (Mat_<float>(4,4) << Pass_R_T11, Pass_R_T12, Pass_R_T13, Pass_R_T14,
									Pass_R_T21, Pass_R_T22, Pass_R_T23, Pass_R_T24,
									Pass_R_T31, Pass_R_T32, Pass_R_T33, Pass_R_T34,
									Pass_R_T41, Pass_R_T42, Pass_R_T43, Pass_R_T44);



int process_frame(Mat img_brut){
	Mat img_topview = Mat(IR_HEIGHT, IR_WIDTH, CV_8UC3);

    // Straightened frame
	int j,i,u,v;
	for(j=0;j<IR_HEIGHT;j++){
		for(i=0;i<IR_WIDTH;i++){
			u = int((3195.*j - 5213.*sqrt(2)*i + 2794168.*sqrt(2) - 3952854.)/(10.*j - 12372.));
			v = int(-(7045.*j - 5151.)/(25.*j - 30930.));

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

	// Selection of a part of frame
		// Center and radius of circle of selection
//				Point2i p_c, p_i;
//				p_c.x = 520;
//				p_c.y = 430;
//				int rad = 160;
//			// Selection of part
//				for(j=0;j<IR_HEIGHT;j++){
//					for(i=0;i<IR_WIDTH;i++){
//						p_i.x = i;
//						p_i.y = j;
//						if(norm(p_c - p_i)>rad){
//							for(int a=0;a<3 ; a++){ //sur les trois canaux
//								img_topview.at<Vec3b>(j,i)[a] = 0;
//							}
//						}
//					}
//				}

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
    	inRange(img_HSV , hsv_min, hsv_max, img_thres_R);


    	erode( img_thres_R, img_thres_R, element );
		dilate( img_thres_R, img_thres_R, element_dilate );


		// Find contours
		std::vector<std::vector<cv::Point> > contours;
		Mat contourOutput = img_thres_R.clone();

		findContours( contourOutput, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );//CV_CHAIN_APPROX_SIMPLE, CV_CHAIN_APPROX_NONE


		//Draw the contours
			vector< vector<Point> > triangles;
			// Array for storing the approximation curve
			cv::Mat contourImage(img_topview.size(), CV_8UC3, cv::Scalar(0,0,0));
			cv::Scalar colors;
			colors = cv::Scalar(0, 0, 255);
			vector<Point> approxTriangle;
			float aires_triangle[contours.size()];
			float perimeter_triangle[contours.size()];
			// Test each contour
			Scalar colorsR;
			for (size_t idx = 0; idx < contours.size(); idx++) { // for each contour
//					cout << "countours #" << idx << ": " << contours[idx] <<endl;

				colorsR = cv::Scalar((idx*30)%256, (idx*30)%256, (idx*30)%256);
				cv::drawContours(img_topview, contours, idx, colorsR,4);


				// approximate contour with accuracy proportional to the contour perimeter
				approxPolyDP(Mat(contours[idx]), approxTriangle, arcLength(cv::Mat(contours[idx]), true) * 0.007, true);

				// Cas du triangle
				// Skip small or non-convex objects
//					printf("\n\tContour:%lu\n\tfabs(contourArea(approxTriangle)) = %f: \n",idx,fabs(contourArea(approxTriangle)));
				// Save area of triangle
				aires_triangle[idx] = fabs(contourArea(approxTriangle));
				perimeter_triangle[idx] = arcLength(approxTriangle,1);
				if ((approxTriangle.size() >= 3 ) /*&& fabs(contourArea(approxTriangle)) > 10000 && isContourConvex(approxTriangle)*/){
//						drawContours(img_topview, contours,idx, colors, 2);
					// Adds elements to the bottom of the matrix
					triangles.push_back(approxTriangle);
				}
				Scalar col = Scalar(255, 255, 0);
	            vector<Point>::iterator vertex;
	            for(vertex = approxTriangle.begin(); vertex != approxTriangle.end(); ++vertex){
	               if(fabs(contourArea(approxTriangle)) > AREA_MIN_DOUBLE_FLAT_TRI){
		            	circle(img_topview, *vertex, 5, col, 2);
	               }

	            }

	            const Point* point = &approxTriangle[0];
				int points = (int) approxTriangle.size();
				if(fabs(contourArea(approxTriangle)) > AREA_MIN_DOUBLE_FLAT_TRI){ 			// MAJ
					polylines(img_topview, &point, &points, 1, true, col, 2, CV_AA);
				}
			} //end: for each contour

			colors = cv::Scalar(0, 0, 255);



			// Identification of each triangle
			for (size_t i=0; i<triangles.size(); i++){ //for each triangle
				char text[32];
				printf("\nContour %lu:\n", i);
				sprintf(text, "#%lu", i);

				Scalar tri_color = Scalar((i*50)%256, (i*80+127)%256, (i*20+40)%256), tri_color2 = Scalar(250, 250, 250);

//					const Point* point = &triangles[i][0];
//					int points = (int) triangles[i].size();
				//printf("\n\tpoints[%lu]=%d",i,points);

//					printf("\t %i points\n", points);

				printf("aires_triangle[%lu]=%f\n",i,aires_triangle[i]);

				if(aires_triangle[i] > AREA_MIN_DOUBLE_FLAT_TRI){
					// get the six longest edges
						int i_tab[6]={-1, -1, -1, -1, -1, -1};
						float tdist[6];
						for(int k=0; k<6; k++){
							tdist[k]= 0.;
						}
						for(int j=0; j<int(triangles[i].size()); j++){
							Point2f p1 = triangles[i][j];
							Point2f p2 = triangles[i][(j+1)%triangles[i].size()];
							float fdist = norm(p1-p2);
//								printf("fdist(%i,%i)=%.2f\n", j, (j+1)%triangles[i].size(), fdist);
//								printf("\n\n j++\n");

							if(!j || fdist > tdist[5]){
								i_tab[0] = i_tab[1];
								i_tab[1] = i_tab[2];
								i_tab[2] = i_tab[3];
								i_tab[3] = i_tab[4];
								i_tab[4] = i_tab[5];
								i_tab[5] = j;
								tdist[0] = tdist[1];
								tdist[1] = tdist[2];
								tdist[2] = tdist[3];
								tdist[3] = tdist[4];
								tdist[4] = tdist[5];
								tdist[5] = fdist;
							}
							else if(fdist > tdist[4]){
								i_tab[0] = i_tab[1];
								i_tab[1] = i_tab[2];
								i_tab[2] = i_tab[3];
								i_tab[3] = i_tab[4];
								i_tab[4] = j;
								tdist[0] = tdist[1];
								tdist[1] = tdist[2];
								tdist[2] = tdist[3];
								tdist[3] = tdist[4];
								tdist[4] = fdist;
							}
							else if(fdist > tdist[3]){
								i_tab[0] = i_tab[1];
								i_tab[1] = i_tab[2];
								i_tab[2] = i_tab[3];
								i_tab[3] = j;
								tdist[0] = tdist[1];
								tdist[1] = tdist[2];
								tdist[2] = tdist[3];
								tdist[3] = fdist;
							}
							else if(fdist > tdist[2]){
								i_tab[0] = i_tab[1];
								i_tab[1] = i_tab[2];
								i_tab[2] = j;
								tdist[0] = tdist[1];
								tdist[1] = tdist[2];
								tdist[2] = fdist;
							}
							else if(fdist > tdist[1]){
								i_tab[0] = i_tab[1];
								i_tab[1] = j;
								tdist[0] = tdist[1];
								tdist[1] =fdist;
							}
							else if(fdist > tdist[0]){
								i_tab[0] = j;
								tdist[0] = fdist;
							}
						}
//							printf("t.size()=%lu\n", triangles[i].size());
//							printf("\n "
//									"i_tab[0] = %d     tdist[0] = %.2fmm\n "
//									"i_tab[1] = %d     tdist[1] = %.2fmm\n "
//									"i_tab[2] = %d     tdist[2] = %.2fmm\n "
//									"i_tab[3] = %d     tdist[3] = %.2fmm\n "
//									"i_tab[4] = %d     tdist[4] = %.2fmm\n "
//									"i_tab[5] = %d     tdist[5] = %.2fmm\n",
//									i_tab[0], tdist[0] * PX2MM, i_tab[1], tdist[1] * PX2MM, i_tab[2], tdist[2] * PX2MM,
//									i_tab[3], tdist[3] * PX2MM, i_tab[4], tdist[4] * PX2MM, i_tab[5], tdist[5] * PX2MM);
//							int cpt=0;
//							for(int k=0; k<6; k++){
//								if(tdist[k] > LONG_MIN_DETECT * MM2PX){
//									cpt++;
//								}
//							}

						printf("\nPerimeter contour %lu = %fmm\n", i, perimeter_triangle[i] *PX2MM);
						if( perimeter_triangle[i] < PERIM_2TRI_CC * MM2PX){
							printf("\n Two flat triangle with the same edges\n");
							// Keep the four longest
							for(int k=0; k<4; k++){
								i_tab[k] = i_tab[k+2];
								tdist[k] = tdist[k+2];
							}
//								printf("\n Keep the four longest\n "
//										"i_tab[0] = %d     tdist[0] = %.2fmm\n "
//										"i_tab[1] = %d     tdist[1] = %.2fmm\n "
//										"i_tab[2] = %d     tdist[2] = %.2fmm\n "
//										"i_tab[3] = %d     tdist[3] = %.2fmm\n ",
//										i_tab[0], tdist[0] * PX2MM, i_tab[1], tdist[1] * PX2MM,
//										i_tab[2], tdist[2] * PX2MM, i_tab[3], tdist[3] * PX2MM);
							// Class by index
							for(int k=0; k<4; k++){ // Keep the four longest edges
								for(int u=(k+1); u<4; u++){
									if(i_tab[k] > i_tab[u]){
										int a = i_tab[u];
										i_tab[u] = i_tab[k];
										i_tab[k] = a;

										float f = tdist[u];
										tdist[u] = tdist[k];
										tdist[k] = f;
									}
								}
							}
//								printf("\n "
//										"i_tab[0] = %d     tdist[0] = %.2fmm\n "
//										"i_tab[1] = %d     tdist[1] = %.2fmm\n "
//										"i_tab[2] = %d     tdist[2] = %.2fmm\n "
//										"i_tab[3] = %d     tdist[3] = %.2fmm\n ",
//										i_tab[0], tdist[0] * PX2MM, i_tab[1], tdist[1] * PX2MM,
//										i_tab[2], tdist[2] * PX2MM, i_tab[3], tdist[3] * PX2MM);
							// calcul des points d'intersections des 6 plus longues droites
								Point2f p1, p2, p3, p4, pm, pi, pv;
								intersection(
									triangles[i][i_tab[0]], triangles[i][(i_tab[0]+1)%triangles[i].size()],
									triangles[i][i_tab[1]], triangles[i][(i_tab[1]+1)%triangles[i].size()],
									p1);
								putText(img_topview, "p1", p1, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
								intersection(
									triangles[i][i_tab[1]], triangles[i][(i_tab[1]+1)%triangles[i].size()],
									triangles[i][i_tab[2]], triangles[i][(i_tab[2]+1)%triangles[i].size()],
									p2);
								putText(img_topview, "p2", p2, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
								intersection(
									triangles[i][i_tab[2]], triangles[i][(i_tab[2]+1)%triangles[i].size()],
									triangles[i][i_tab[3]], triangles[i][(i_tab[3]+1)%triangles[i].size()],
									p3);
								putText(img_topview, "p3", p3, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
								intersection(
									triangles[i][i_tab[3]], triangles[i][(i_tab[3]+1)%triangles[i].size()],
									triangles[i][i_tab[0]], triangles[i][(i_tab[0]+1)%triangles[i].size()],
									p4);
								putText(img_topview, "p4", p4, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);

								printf("\n"
										"\t\t p1 = (%.2fmm; %.2fmm)\n"
										"\t\t p2 = (%.2fmm; %.2fmm)\n"
										"\t\t p3 = (%.2fmm; %.2fmm)\n"
										"\t\t p4 = (%.2fmm; %.2fmm)\n\n",
										p1.x * PX2MM, p1.y * PX2MM, p2.x * PX2MM, p2.y * PX2MM,
										p3.x * PX2MM, p3.y * PX2MM, p4.x * PX2MM, p4.y * PX2MM);
							// Trace points
								circle(img_topview, p1, 6, tri_color, 2);
								circle(img_topview, p2, 6, tri_color, 2);
								circle(img_topview, p3, 6, tri_color, 2);
								circle(img_topview, p4, 6, tri_color, 2);

								Scalar tri_color3 = Scalar(0, 0, 250);
								line(img_topview, p1, p2, tri_color3, 2);
								line(img_topview, p2, p3, tri_color3, 2);
								line(img_topview, p3, p4, tri_color3, 2);
								line(img_topview, p4, p1, tri_color3, 2);

								pm = (p1+p2+p3+p4)*(1./4.);
								circle(img_topview, pm, 6, tri_color, 2);
								putText(img_topview, text, pm, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);

								sprintf(text, "%.2fmm", norm(p1-p2)*PX2MM);
								putText(img_topview, text, 0.5*(p1+p2), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
								sprintf(text, "%.2fmm", norm(p2-p3)*PX2MM);
								putText(img_topview, text, 0.5*(p2+p3), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
								sprintf(text, "%.2fmm", norm(p3-p4)*PX2MM);
								putText(img_topview, text, 0.5*(p3+p4), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
								sprintf(text, "%.2fmm", norm(p4-p1)*PX2MM);
								putText(img_topview, text, 0.5*(p4+p1), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);

							// Identification of two triangles
								// Angle between p4, p1, p2
									float alpha = acos( (pow(norm(p4-p2), 2) - pow(norm(p1-p2), 2) - pow(norm(p1-p4), 2)) / (-2 * norm(p1-p2) * norm(p1-p4)));
//									printf("\n\t alpha = %.2f°\n", alpha * RAD2DEG);
								if((alpha * RAD2DEG) > (ANGL_MIN_FLAT_TRI * 2) && (alpha * RAD2DEG) < (ANGL_MAX_FLAT_TRI * 2)){ // on two vertices on p1
									line(img_topview, p3, p1, tri_color3, 2);
									// First triangle
										// Calcul center of triangle
											pm = (p1 + p4 + p3)*(1./3.);
										// Calcul de theta_feu
											pi.x = (p1.x+p3.x)/2;
											pi.y = (p1.y+p3.y)/2;
											pv.x = p4.x - pi.x;
											pv.y = p4.y - pi.y;
											float theta = atan2(pv.y,pv.x)*180/M_PI;
										// Draw direction of fire
											line(img_topview, pi, p4, tri_color, 3);
											circle(img_topview, pi, 10, tri_color, 3);
										// Normalization of theta (the smallest  positive angle relative axis y of primary robot)
			//										printf("pi.x = %f; pi.y = %f\n", pi.x, pi.y);
			//										printf("p3.x = %f; p3.y = %f\n", p3.x, p3.y);
			//										printf("pv.x = %f; pv.y = %f\n", pv.x, pv.y);

										theta = -fmodf(theta, 120.);
										if(theta < 0){
											theta = theta +120;
										}

										if(nb_fx_frame < NBR_FEUX){
											Infos_feux_cap[nb_fx_frame].x_feu = I2R(pm).x;
											Infos_feux_cap[nb_fx_frame].y_feu = I2R(pm).y;
											Infos_feux_cap[nb_fx_frame].theta_feu = theta;
											Infos_feux_cap[nb_fx_frame].coul_feu = Rouge;
											Infos_feux_cap[nb_fx_frame].etat_feu = Horizontal;

											printf("\t\t\tInfos_feux_cap[%d].x_feu = %.2f mm\n"
													"\t\t\tInfos_feux_cap[%d].y_feu = %.2f mm\n"
													"\t\t\tInfos_feux_cap[%d].theta_feu = %.2f deg\n"
													"\t\t\tInfos_feux_cap[%d].coul_feu = Rouge\n"
													"\t\t\tInfos_feux_cap[%d].etat_feu = Horizontal\n",
													nb_fx_frame,Infos_feux_cap[nb_fx_frame].x_feu,nb_fx_frame,
													Infos_feux_cap[nb_fx_frame].y_feu, nb_fx_frame, Infos_feux_cap[nb_fx_frame].theta_feu,
													nb_fx_frame, nb_fx_frame);
											nb_fx_frame++;
											printf("\t\tnb_fx_frame = %d\n\n", nb_fx_frame);

										}
										else{
											printf("\nnb_fx_frame=%d\nCan't add any more fire in temporary struct\n", nb_fx_frame);
										}
									// Second triangle
										// Calcul center of triangle
											pm = (p1 + p2 + p3)*(1./3.);
										// Calcul de theta_feu
											pi.x = (p1.x+p3.x)/2;
											pi.y = (p1.y+p3.y)/2;
											pv.x = p2.x - pi.x;
											pv.y = p2.y - pi.y;
											theta = atan2(pv.y,pv.x)*180/M_PI;
										// Draw direction of fire
											line(img_topview, pi, p2, tri_color, 3);
											circle(img_topview, pi, 10, tri_color, 3);
										// Normalization of theta (the smallest  positive angle relative axis y of primary robot)
			//										printf("pi.x = %f; pi.y = %f\n", pi.x, pi.y);
			//										printf("p3.x = %f; p3.y = %f\n", p3.x, p3.y);
			//										printf("pv.x = %f; pv.y = %f\n", pv.x, pv.y);

										theta = -fmodf(theta, 120.);
										if(theta < 0){
											theta = theta +120;
										}
										if(nb_fx_frame < NBR_FEUX){
											Infos_feux_cap[nb_fx_frame].x_feu = I2R(pm).x;
											Infos_feux_cap[nb_fx_frame].y_feu = I2R(pm).y;
											Infos_feux_cap[nb_fx_frame].theta_feu = theta;
											Infos_feux_cap[nb_fx_frame].coul_feu = Rouge;
											Infos_feux_cap[nb_fx_frame].etat_feu = Horizontal;
											printf("\t\t\tInfos_feux_cap[%d].x_feu = %.2f mm\n"
													"\t\t\tInfos_feux_cap[%d].y_feu = %.2f mm\n"
													"\t\t\tInfos_feux_cap[%d].theta_feu = %.2f deg\n"
													"\t\t\tInfos_feux_cap[%d].coul_feu = Rouge\n"
													"\t\t\tInfos_feux_cap[%d].etat_feu = Horizontal\n",
													nb_fx_frame,Infos_feux_cap[nb_fx_frame].x_feu,nb_fx_frame,
													Infos_feux_cap[nb_fx_frame].y_feu, nb_fx_frame, Infos_feux_cap[nb_fx_frame].theta_feu,
													nb_fx_frame, nb_fx_frame);
											nb_fx_frame++;
											printf("\t\tnb_fx_frame = %d\n\n", nb_fx_frame);

										}
										else{
											printf("\nnb_fx_frame=%d\nCan't add any more fire in temporary struct\n", nb_fx_frame);
										}

								}// end: if((alpha * RAD2DEG) > (ANGL_MIN_FLAT_TRI * 2) && (alpha * RAD2DEG) < (ANGL_MAX_FLAT_TRI * 2))

								else if((alpha * RAD2DEG) > ANGL_MIN_FLAT_TRI  && (alpha * RAD2DEG) < ANGL_MAX_FLAT_TRI){// one vertices on p1
									line(img_topview, p2, p4, tri_color3, 2);
									// First triangle
											// Calcul center of triangle
												pm = (p1 + p4 + p2)*(1./3.);
											// Calcul de theta_feu
												pi.x = (p1.x+p2.x)/2;
												pi.y = (p1.y+p2.y)/2;
												pv.x = p4.x - pi.x;
												pv.y = p4.y - pi.y;
												float theta = atan2(pv.y,pv.x)*180/M_PI;
											// Draw direction of fire
												line(img_topview, pi, p4, tri_color, 3);
												circle(img_topview, pi, 10, tri_color, 3);
											// Normalization of theta (the smallest  positive angle relative axis y of primary robot)
				//										printf("pi.x = %f; pi.y = %f\n", pi.x, pi.y);
				//										printf("p3.x = %f; p3.y = %f\n", p3.x, p3.y);
				//										printf("pv.x = %f; pv.y = %f\n", pv.x, pv.y);

											theta = -fmodf(theta, 120.);
											if(theta < 0){
												theta = theta +120;
											}

											Infos_feux_cap[nb_fx_frame].x_feu = I2R(pm).x;
											Infos_feux_cap[nb_fx_frame].y_feu = I2R(pm).y;
											Infos_feux_cap[nb_fx_frame].theta_feu = theta;
											Infos_feux_cap[nb_fx_frame].coul_feu = Rouge;
											Infos_feux_cap[nb_fx_frame].etat_feu = Horizontal;
											printf("\t\t\tInfos_feux_cap[%d].x_feu = %.2f mm\n"
													"\t\t\tInfos_feux_cap[%d].y_feu = %.2f mm\n"
													"\t\t\tInfos_feux_cap[%d].theta_feu = %.2f deg\n"
													"\t\t\tInfos_feux_cap[%d].coul_feu = Rouge\n"
													"\t\t\tInfos_feux_cap[%d].etat_feu = Horizontal\n\n",
													nb_fx_frame ,Infos_feux_cap[nb_fx_frame].x_feu,nb_fx_frame, Infos_feux_cap[nb_fx_frame].y_feu,
													nb_fx_frame, Infos_feux_cap[nb_fx_frame].theta_feu, nb_fx_frame, nb_fx_frame);
											nb_fx_frame++;
											printf("\t\tnb_fx_frame = %d\n\n", nb_fx_frame);
										// Second triangle
											// Calcul center of triangle
												pm = (p4 + p2 + p3)*(1./3.);
											// Calcul de theta_feu
												pi.x = (p2.x+p3.x)/2;
												pi.y = (p2.y+p3.y)/2;
												pv.x = p4.x - pi.x;
												pv.y = p4.y - pi.y;
												theta = atan2(pv.y,pv.x)*180/M_PI;
											// Draw direction of fire
												line(img_topview, pi, p4, tri_color, 3);
												circle(img_topview, pi, 10, tri_color, 3);
											// Normalization of theta (the smallest  positive angle relative axis y of primary robot)
				//										printf("pi.x = %f; pi.y = %f\n", pi.x, pi.y);
				//										printf("p3.x = %f; p3.y = %f\n", p3.x, p3.y);
				//										printf("pv.x = %f; pv.y = %f\n", pv.x, pv.y);

											theta = -fmodf(theta, 120.);
											if(theta < 0){
												theta = theta +120;
											}

											Infos_feux_cap[nb_fx_frame].x_feu = I2R(pm).x;
											Infos_feux_cap[nb_fx_frame].y_feu = I2R(pm).y;
											Infos_feux_cap[nb_fx_frame].theta_feu = theta;
											Infos_feux_cap[nb_fx_frame].coul_feu = Rouge;
											Infos_feux_cap[nb_fx_frame].etat_feu = Horizontal;
											printf("\t\t\tInfos_feux_cap[%d].x_feu = %.2f mm\n"
													"\t\t\tInfos_feux_cap[%d].y_feu = %.2f mm\n"
													"\t\t\tInfos_feux_cap[%d].theta_feu = %.2f deg\n"
													"\t\t\tInfos_feux_cap[%d].coul_feu = Rouge\n"
													"\t\t\tInfos_feux_cap[%d].etat_feu = Horizontal\n\n",
													nb_fx_frame ,Infos_feux_cap[nb_fx_frame].x_feu,nb_fx_frame, Infos_feux_cap[nb_fx_frame].y_feu,
													nb_fx_frame, Infos_feux_cap[nb_fx_frame].theta_feu, nb_fx_frame, nb_fx_frame);
											nb_fx_frame++;
											printf("\t\tnb_fx_frame = %d\n\n", nb_fx_frame);
								}
								else{
									printf("\n Mauvaise detection\n");
								}
						} //end :if( perimeter_triangle[i] < PERIM_2TRI_CC * MM2PX)
//
//							else if( perimeter_triangle[i] > PERIM_2TRI_CC){
//								// Class by index
//								for(int k=0; k<6; k++){
//									for(int u=(k+1); u<6; u++){
//										if(i_tab[k] > i_tab[u]){
//											int a = i_tab[u];
//											i_tab[u] = i_tab[k];
//											i_tab[k] = a;
//
//											float f = tdist[u];
//											tdist[u] = tdist[k];
//											tdist[k] = f;
//										}
//									}
//								}
//
//							// calcul des points d'intersections des 6 plus longues droites
//								Point2f p1, p2, p3, p4, p5, p6, pm, pi, pv;
//								intersection(
//									triangles[i][i_tab[0]], triangles[i][(i_tab[0]+1)%triangles[i].size()],
//									triangles[i][i_tab[1]], triangles[i][(i_tab[1]+1)%triangles[i].size()],
//									p1);
//								putText(img_topview, "p1", p1, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
//								intersection(
//									triangles[i][i_tab[1]], triangles[i][(i_tab[1]+1)%triangles[i].size()],
//									triangles[i][i_tab[2]], triangles[i][(i_tab[2]+1)%triangles[i].size()],
//									p2);
//								putText(img_topview, "p2", p2, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
//								intersection(
//									triangles[i][i_tab[2]], triangles[i][(i_tab[2]+1)%triangles[i].size()],
//									triangles[i][i_tab[3]], triangles[i][(i_tab[3]+1)%triangles[i].size()],
//									p3);
//								putText(img_topview, "p3", p3, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
//								intersection(
//									triangles[i][i_tab[3]], triangles[i][(i_tab[3]+1)%triangles[i].size()],
//									triangles[i][i_tab[4]], triangles[i][(i_tab[4]+1)%triangles[i].size()],
//									p4);
//								putText(img_topview, "p4", p4, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
//								intersection(
//									triangles[i][i_tab[4]], triangles[i][(i_tab[4]+1)%triangles[i].size()],
//									triangles[i][i_tab[5]], triangles[i][(i_tab[5]+1)%triangles[i].size()],
//									p5);
//								putText(img_topview, "p5", p5, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
//								intersection(
//									triangles[i][i_tab[5]], triangles[i][(i_tab[5]+1)%triangles[i].size()],
//									triangles[i][i_tab[0]], triangles[i][(i_tab[0]+1)%triangles[i].size()],
//									p6);
//								putText(img_topview, "p6", p6, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
//
//								printf("\n p1=(%.2fmm; %.2fmm)\n p2=(%.2fmm; %.2fmm)\n p3=(%.2fmm; %.2fmm)\n p4=(%.2fmm; %.2fmm)\n"
//										" p5=(%.2fmm; %.2fmm)\n p6=(%.2fmm; %.2fmm)\n",
//										p1.x * PX2MM, p1.y * PX2MM, p2.x * PX2MM, p2.y * PX2MM, p3.x * PX2MM, p3.y * PX2MM, p4.x * PX2MM, p4.y * PX2MM,
//										p5.x * PX2MM, p5.y * PX2MM, p6.x * PX2MM, p6.y * PX2MM);
//
//								circle(img_topview, p1, 6, tri_color, 2);
//								circle(img_topview, p2, 6, tri_color, 2);
//								circle(img_topview, p3, 6, tri_color, 2);
//								circle(img_topview, p4, 6, tri_color, 2);
//								circle(img_topview, p5, 7, tri_color, 2);
//								circle(img_topview, p6, 8, tri_color, 3);
//								line(img_topview, p1, p2, tri_color, 2);
//								line(img_topview, p2, p3, tri_color, 2);
//								line(img_topview, p3, p4, tri_color, 2);
//								line(img_topview, p4, p5, tri_color, 2);
//								line(img_topview, p5, p6, tri_color, 2);
//								line(img_topview, p6, p1, tri_color, 2);
//								pm = (p1+p2+p3+p4+p5+p6)*(1./6.);
//								circle(img_topview, pm, 6, tri_color, 2);
//								putText(img_topview, text, pm, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
//
//		//							printf("\t d1 : %.2fmm \t %.2fpx\n", 1210.*norm(p1-p2)/1671., norm(p1-p2));
//		//							printf("\t d2 : %.2fmm \t %.2fpx\n", 1210.*norm(p2-p3)/1671., norm(p2-p3));
//		//							printf("\t d3 : %.2fmm \t %.2fpx\n", 1210.*norm(p3-p1)/1671., norm(p3-p1));
//		//							printf("\t centre %.2f;%.2fmm\n", pm.x, pm.y);
//								sprintf(text, "%.2fmm", norm(p1-p2)*PX2MM);
//								putText(img_topview, text, 0.5*(p1+p2), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
//								sprintf(text, "%.2fmm", norm(p2-p3)*PX2MM);
//								putText(img_topview, text, 0.5*(p2+p3), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
//								sprintf(text, "%.2fmm", norm(p3-p4)*PX2MM);
//								putText(img_topview, text, 0.5*(p3+p4), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
//								sprintf(text, "%.2fmm", norm(p4-p5)*PX2MM);
//								putText(img_topview, text, 0.5*(p4+p5), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
//								sprintf(text, "%.2fmm", norm(p5-p6)*PX2MM);
//								putText(img_topview, text, 0.5*(p4+p6), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
//								sprintf(text, "%.2fmm", norm(p6-p1)*PX2MM);
//								putText(img_topview, text, 0.5*(p6+p1), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
//							}
				}
				else{
					// get the three longest edges
						int i_min=-1, i_med=-1, i_max=-1;
						float dist_min=-1, dist_med=-1, dist_max=-1;
						for(int j=0; j<int(triangles[i].size()); j++){
							Point2f p1 = triangles[i][j];
							Point2f p2 = triangles[i][(j+1)%triangles[i].size()];
							float dist = norm(p1-p2);

							if(!j || dist > dist_max){
								i_min = i_med;
								i_med = i_max;
								i_max = j;
								dist_min = dist_med;
								dist_med = dist_max;
								dist_max = dist;
							}
							else if(dist > dist_med){
								i_min = i_med;
								i_med = j;
								dist_min = dist_med;
								dist_med = dist;
							}
							else if(dist > dist_min){
								i_min = j;
								dist_min = dist;
							}
						}
						if(dist_min > (MM2PX * LONG_MIN_DETECT)){


		//						printf("\t\t trois plus grandes cotes: max-2=%.2f px; max-1=%.2fpx; max-0=%.2fpx\n", dist_min, dist_med, dist_max);

							// calcul des points d'intersections de ces trois plus longues droites
								Point2f p1, p2, p3, pm, pi, pv;
								intersection(
									triangles[i][i_min], triangles[i][(i_min+1)%triangles[i].size()],
									triangles[i][i_med], triangles[i][(i_med+1)%triangles[i].size()],
									p1);
								putText(img_topview, "p1", p1, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
								intersection(
									triangles[i][i_med], triangles[i][(i_med+1)%triangles[i].size()],
									triangles[i][i_max], triangles[i][(i_max+1)%triangles[i].size()],
									p2);
								putText(img_topview, "p2", p2, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);
								intersection(
									triangles[i][i_max], triangles[i][(i_max+1)%triangles[i].size()],
									triangles[i][i_min], triangles[i][(i_min+1)%triangles[i].size()],
									p3);
								putText(img_topview, "p3", p3, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);

								circle(img_topview, p1, 6, tri_color, 2);
								circle(img_topview, p2, 6, tri_color, 2);
								circle(img_topview, p3, 7, tri_color, 3);
								line(img_topview, p1, p2, tri_color, 2);
								line(img_topview, p2, p3, tri_color, 2);
								line(img_topview, p3, p1, tri_color, 2);
								pm = (p1+p2+p3)*(1./3.);
								circle(img_topview, pm, 6, tri_color, 2);
								putText(img_topview, text, pm, FONT_HERSHEY_SIMPLEX, 1./2., tri_color, 1);

		//							printf("\t d1 : %.2fmm \t %.2fpx\n", 1210.*norm(p1-p2)/1671., norm(p1-p2));
		//							printf("\t d2 : %.2fmm \t %.2fpx\n", 1210.*norm(p2-p3)/1671., norm(p2-p3));
		//							printf("\t d3 : %.2fmm \t %.2fpx\n", 1210.*norm(p3-p1)/1671., norm(p3-p1));
		//							printf("\t centre %.2f;%.2fmm\n", pm.x, pm.y);
								sprintf(text, "%.2fmm", norm(p1-p2)*1210./1671.);
								putText(img_topview, text, 0.5*(p1+p2), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
								sprintf(text, "%.2fmm", norm(p2-p3)*1210./1671.);
								putText(img_topview, text, 0.5*(p2+p3), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);
								sprintf(text, "%.2fmm", norm(p3-p1)*1210./1671.);
								putText(img_topview, text, 0.5*(p3+p1), FONT_HERSHEY_SIMPLEX, 1./2., tri_color2, 1);

						/////////////////////////////////////////////////////
						//// for an area of isolated flat entire triangle
						/////////////////////////////////////////////////////
						if(aires_triangle[i] >= AREA_MIN_FLAT_TRI && aires_triangle[i] <= AREA_MAX_FLAT_TRI){
							if(norm(p1-p2) > LONG_TRI_MIN*1671./1210. && norm(p1-p2) < LONG_TRI_MAX*1671./1210. &&
							   norm(p2-p3) > LONG_TRI_MIN*1671./1210. && norm(p2-p3) < LONG_TRI_MAX*1671./1210. &&
							   norm(p3-p1) > LONG_TRI_MIN*1671./1210. && norm(p3-p1) < LONG_TRI_MAX*1671./1210. ){
		//							printf("\t Triangle à plat\n");
								// Calcul de theta_feu
									pi.x = (p1.x+p2.x)/2;
									pi.y = (p1.y+p2.y)/2;
									pv.x = p3.x - pi.x;
									pv.y = p3.y - pi.y;
									float theta = atan2(pv.y,pv.x)*180/M_PI;
									// Draw direction of fire
										line(img_topview, pi, p3, tri_color, 3);
										circle(img_topview, pi, 10, tri_color, 3);
									// Normalization of theta (the smallest  positive angle relative axis y of primary robot)
		//										printf("pi.x = %f; pi.y = %f\n", pi.x, pi.y);
		//										printf("p3.x = %f; p3.y = %f\n", p3.x, p3.y);
		//										printf("pv.x = %f; pv.y = %f\n", pv.x, pv.y);

								theta = -fmodf(theta, 120.);
								if(theta < 0){
									theta = theta +120;
								}

//									Infos_feux_cap[i].x_feu = I2R(pm).x;
//									Infos_feux_cap[i].y_feu = I2R(pm).y;
//									Infos_feux_cap[i].theta_feu = theta;
//									Infos_feux_cap[i].coul_feu = Rouge;
//									Infos_feux_cap[i].etat_feu = Horizontal;
//									printf("\t\t\tInfos_feux_cap[%lu].x_feu = %.2f mm\n"
//											"\t\t\tInfos_feux_cap[%lu].y_feu = %.2f mm\n"
//											"\t\t\tInfos_feux_cap[%lu].theta_feu = %.2f deg\n"
//											"\t\t\tInfos_feux_cap[%lu].coul_feu = Rouge\n"
//											"\t\t\tInfos_feux_cap[%lu].etat_feu = Horizontal\n\n",i,Infos_feux_cap[i].x_feu,i,
//											Infos_feux_cap[i].y_feu, i, Infos_feux_cap[i].theta_feu, i, i);
							}
							i++;
						} //end if (area flat triangle)

						//////////////////////////////////////////////////////////////////
						//// for a vertical triangle
						//////////////////////////////////////////////////////////////////
//							if((norm(p1-p2) > LONG_TRI_VERTI_MIN*1671./1210. && norm(p1-p2) < LONG_TRI_VERTI_MAX*1671./1210.) ||
//							   (norm(p2-p3) > LONG_TRI_VERTI_MIN*1671./1210. && norm(p2-p3) < LONG_TRI_VERTI_MAX*1671./1210.) ||
//							   (norm(p3-p1) > LONG_TRI_VERTI_MIN*1671./1210. && norm(p3-p1) < LONG_TRI_VERTI_MAX*1671./1210.)){
//								// Calcul of angle between lines
//								float alpha, beta, gamma, zeta;
//								alpha = acos( (pow(norm(p1-p2),2)-pow(norm(p2-p3),2)-pow(norm(p3-p1),2)) / (-2*norm(p3-p1)*norm(p2-p3)) )*180/M_PI;
//								beta  = acos( (pow(norm(p2-p3),2)-pow(norm(p1-p2),2)-pow(norm(p3-p1),2)) / (-2*norm(p3-p1)*norm(p1-p2)) )*180/M_PI;
//								gamma = acos( (pow(norm(p3-p1),2)-pow(norm(p1-p2),2)-pow(norm(p2-p3),2)) / (-2*norm(p2-p3)*norm(p1-p2)) )*180/M_PI;
//			//						printf("\t\t Angle entre:\n"
//			//								"\t\t\tangle(p1,p3,p2): alpha = %f\n"
//			//								"\t\t\tangle(p2,p1,p3): beta = %f\n"
//			//								"\t\t\tangle(p3,p2,p1): gamma = %f\n", alpha, beta, gamma);
//								int long_tri_table = 0;
//								if(norm(p1-p2) > LONG_TRI_VERTI_MIN*1671./1210. && norm(p1-p2) < LONG_TRI_VERTI_MAX*1671./1210.){
//									long_tri_table = norm(p1-p2);
//			//							printf("\np1p2\n");
//									// Update angles
//										zeta = alpha;
//										alpha = beta;
//										beta = gamma;
//										gamma = zeta;
//									// Update centre
//										pm = 0.5*(p1+p2);
//									// Orientation of triangle
//										pi.x = -(p1.y-p2.y) / norm(p1-p2);
//										pi.y = (p1.x-p2.x) / norm(p1-p2);
//										if((p2.x-p1.x)*(p3.y-pm.y)-(p2.y-p1.y)*(p3.x-pm.x) < 0){
//											pi.x = -pi.x;
//											pi.y = -pi.y;
//										}
//								}
//								else if(norm(p2-p3) > LONG_TRI_VERTI_MIN*1671./1210. && norm(p2-p3) < LONG_TRI_VERTI_MAX*1671./1210.){
//									long_tri_table = norm(p2-p3);
//			//							printf("\np3p2\n");
//									// Update angles
//										zeta = alpha;
//										alpha = gamma;
//										gamma = beta;
//										beta = zeta;
//									// Update centre
//										pm = 0.5*(p3+p2);
//									// Orientation of triangle
//										pi.x = -(p2.y-p3.y) / norm(p2-p3);
//										pi.y = (p2.x-p3.x) / norm(p2-p3);
//										if((p3.x-p2.x)*(p1.y-pm.y)-(p3.y-p2.y)*(p1.x-pm.x) < 0){
//											pi.x = -pi.x;
//											pi.y = -pi.y;
//										}
//								}
//								else if(norm(p3-p1) > LONG_TRI_VERTI_MIN*1671./1210. && norm(p3-p1) < LONG_TRI_VERTI_MAX*1671./1210.){
//									long_tri_table = norm(p3-p1);
//			//							printf("\np1p3\n");
//									//not need update angles
//									// Update centre
//										pm = 0.5*(p3+p1);
//									// Orientation of triangle
//										pi.x = -(p3.y-p1.y)/norm(p3-p1);
//										pi.y = (p3.x-p1.x) / norm(p3-p1);
//										if((p1.x-p3.x)*(p2.y-pm.y)-(p1.y-p3.y)*(p2.x-pm.x) < 0){
//											pi.x = -pi.x;
//											pi.y = -pi.y;
//										}
//								}
//
//								if(long_tri_table != 0){
//									pi = 30.*pi + pm;
//									zeta = -atan2((pi.y-pm.y), (pi.x-pm.x))*180./M_PI;
//
//			//							printf("\npi.x=%.2f, pi.y=%.2f\npm.x=%.2f, pm.y=%.2f\n(pi-pm).x=%.2f, (pi-pm).y=%.2f\n",
//			//									pi.x, pi.y,pm.x, pm.y, (pi.x-pm.x), (pi.y-pm.y));
//									// Draw direction of fire
//										line(img_topview, pm, pi, tri_color, 3);
//										circle(img_topview, pi, 8, tri_color, 2);
//									Infos_feux_cap[i].x_feu = I2R(pm).x; //pm.x*1210./1671.;
//									Infos_feux_cap[i].y_feu = I2R(pm).y; //pm.y*1210./1671.;
//									Infos_feux_cap[i].theta_feu = zeta;
//									Infos_feux_cap[i].coul_feu = Rouge;
//									Infos_feux_cap[i].etat_feu = Vertical;
//									printf("\t\t\tInfos_feux_cap[%lu].x_feu = %.2f mm\n"
//											"\t\t\tInfos_feux_cap[%lu].y_feu = %.2f mm\n"
//											"\t\t\tInfos_feux_cap[%lu].theta_feu = %.2f deg\n"
//											"\t\t\tInfos_feux_cap[%lu].coul_feu = Rouge\n"
//											"\t\t\tInfos_feux_cap[%lu].etat_feu = Vertical\n\n",i,Infos_feux_cap[i].x_feu,i,
//											Infos_feux_cap[i].y_feu, i, Infos_feux_cap[i].theta_feu, i, i);
//								}
//
//
//
//							}

						//////////////////////////////////////////////////////////////////
						//// for an area inferior at the one of the isolated flat triangle
						//////////////////////////////////////////////////////////////////
		//					if(aires_triangle[i] < AREA_MIN_FLAT_TRI){
		//						// Calcul of angle between lines
		//							float alpha, beta, gamma;
		//							alpha = acos( (pow(norm(p1-p2),2)-pow(norm(p2-p3),2)-pow(norm(p3-p1),2)) / (-2*norm(p3-p1)*norm(p2-p3)) )*180/M_PI;
		//							beta  = acos( (pow(norm(p2-p3),2)-pow(norm(p1-p2),2)-pow(norm(p3-p1),2)) / (-2*norm(p3-p1)*norm(p1-p2)) )*180/M_PI;
		//							gamma = acos( (pow(norm(p3-p1),2)-pow(norm(p1-p2),2)-pow(norm(p2-p3),2)) / (-2*norm(p2-p3)*norm(p1-p2)) )*180/M_PI;
		//							printf("\t\t Angle entre:\n"
		//									"\t\t\tangle(p1,p3,p2): alpha = %f\n"
		//									"\t\t\tangle(p2,p1,p3): beta = %f\n"
		//									"\t\t\tangle(p3,p2,p1): gamma = %f\n", alpha, beta, gamma);
		//
		//						// triangle on an edge of the frame
		//						if( (((norm(p1-p2) > LONG_TRI_MIN*1671./1210.) && (norm(p1-p2) < LONG_TRI_MAX*1671./1210.)) ||
		//						 ((norm(p2-p3) > LONG_TRI_MIN*1671./1210.) && (norm(p2-p3) < LONG_TRI_MAX*1671./1210.)) ||
		//						 ((norm(p3-p1) > LONG_TRI_MIN*1671./1210.) && (norm(p3-p1) < LONG_TRI_MAX*1671./1210.)))
		//						 &&
		//						 ((alpha > 50 && alpha < 70) || (beta < 50 && beta > 70) || (gamma > 50 && gamma < 70)) ){
		//							// Construction of the entire triangle
		//							if(norm(p1-p2) > LONG_TRI_MIN*1671./1210. && norm(p1-p2) < LONG_TRI_MAX*1671./1210.){
		//								Point2f p3_anc = p3;
		//								// à continuer
		//							}
		//						}
		//
		//					} // end area inferior at the one of the flat isolated triangle





		/*					// draw contour
		//					polylines(img_topview, &point, &points, 1, true, tri_color, 3, CV_AA);

						// find minimum enclosing circle
							Point2f center;
							float radius;
							minEnclosingCircle(Mat(triangles[i]), center, radius);
		//					circle(img_topview, center, 3, tri_color, 3);
		//					printf("  centre cercle circonscrit %.2f,%.2fpx rayon %.2fpx\n", center.x, center.y, radius);
							putText(img_topview, text, center, FONT_HERSHEY_SIMPLEX, 1., tri_color, 2);

						// draw it
		//					Scalar col = Scalar(0, 255, 255);
		//	                circle(img_topview, center, radius, tri_color, 3);

						// displaying vertexes coordinates in robot frame in mm
							printf("  coordonnées sommets dans repère robot:\n");
							float xx =  0;
							float yy =  0;
							float xx_px =  0;
							float yy_px =  0;
							float nb = points;
							for(;points>0;points--){
								xx_px += point->x;
								yy_px += point->y;

								float x =  (1210.*point->x)/1671. - 648560./1671.;
								float y =  423809./557. - (1210.*point->y)/1671.;
								xx += x;
								yy += y;
		//						printf("    [%03.2fmm ; %03.2fmm]\n", x, y);
								point++;
							}
		//					printf("  barycentre [%03.2fmm ; %03.2fmm]\n", xx/nb, yy/nb);
		//					circle(img_topview, Point2f(xx_px/nb, yy_px/nb), 6, tri_color, 2);


						// get the three longest edges
							int i_min=-1, i_med=-1, i_max=-1;
							float dist_min=-1, dist_med=-1, dist_max=-1;
							for(int j=0; j<int(triangles[i].size()); j++){
								Point2f p1 = triangles[i][j];
								Point2f p2 = triangles[i][(j+1)%triangles[i].size()];
								float dist = norm(p1-p2);

								if(!j || dist > dist_max){
									i_min = i_med;
									i_med = i_max;
									i_max = j;
									dist_min = dist_med;
									dist_med = dist_max;
									dist_max = dist;
								}
								else if(dist > dist_med){
									i_min = i_med;
									i_med = j;
									dist_min = dist_med;
									dist_med = dist;
								}
								else if(dist > dist_min){
									i_min = j;
									dist_min = dist;
								}
							}
							printf("max-2 : %.2f\t\t%.2f\n", dist_min, dist_min*1210./1671.);
							printf("max-1 : %.2f\t\t%.2f\n", dist_med, dist_min*1210./1671.);
							printf("max-0 : %.2f\t\t%.2f\n", dist_max, dist_min*1210./1671.);



						// calcul des points d'intersections de ces trois plus longues droites
						{
							Point2f p1, p2, p3, pm;

							intersection(
								triangles[i][i_min], triangles[i][(i_min+1)%triangles[i].size()],
								triangles[i][i_med], triangles[i][(i_med+1)%triangles[i].size()],
								p1
							);
							intersection(
								triangles[i][i_med], triangles[i][(i_med+1)%triangles[i].size()],
								triangles[i][i_max], triangles[i][(i_max+1)%triangles[i].size()],
								p2
							);
							intersection(
								triangles[i][i_max], triangles[i][(i_max+1)%triangles[i].size()],
								triangles[i][i_min], triangles[i][(i_min+1)%triangles[i].size()],
								p3
							);

							circle(img_topview, p1, 6, tri_color, 2);
							circle(img_topview, p2, 6, tri_color, 2);
							circle(img_topview, p3, 6, tri_color, 2);

							printf("\tPosition sommets:\n"
									"\t\tp1.x = %f\t\tp1.y = %f\n"
									"\t\tp2.x = %f\t\tp2.y = %f\n"
									"\t\tp3.x = %f\t\tp3.y = %f\n", p1.x, p1.y,p2.x, p2.y,p3.x, p3.y);
							printf("\t d1 : %.2fmm \t %.2fpx\n", 1210.*norm(p1-p2)/1671., norm(p1-p2));
							printf("\t d2 : %.2fmm \t %.2fpx\n", 1210.*norm(p2-p3)/1671., norm(p2-p3));
							printf("\t d3 : %.2fmm \t %.2fpx\n", 1210.*norm(p3-p1)/1671., norm(p3-p1));

							line(img_topview, p1, p2, tri_color, 2);
							line(img_topview, p2, p3, tri_color, 2);
							line(img_topview, p3, p1, tri_color, 2);

							pm = (p1+p2+p3)*(1./3.);

							printf("\t centre %.2f;%.2fmm\n", pm.x, pm.y);

							circle(img_topview, pm, 6, tri_color, 2);
						}
		*/
						}
					} // end: if(dist_min < (MM2PX * LONG_MIN_DETECT))
			} // end:for (size_t i=0; i<triangles.size(); i++)



    // Show the frame in "MyVideo" window
    imshow("MyVideo", img_brut);
	imshow("brute redressee", img_topview);
    imshow("Video_thresh_R",img_thres_R);
//       imshow("Video_thresh_J",img_thres_J);

    //wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
    if(waitKey(30) == 27){
		cout << "esc key is pressed by user" << endl;
		return -1;
    }
    return 0;
}





