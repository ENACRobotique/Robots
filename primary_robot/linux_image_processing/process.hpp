/*
 * process.hpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#ifndef PROCESS_HPP_
#define PROCESS_HPP_

#include "tools.hpp"


typedef enum {
	Rouge, Jaune, Autre_Coul
}Coul_Feu;



void on_trackbar(int, void*);
int process_frame(Mat img_brut);


extern Scalar hsv_min,hsv_max;

/// Global Variables
extern const int h_slider_max, sv_slider_max;
extern int hmin_slider, hmax_slider, smin_slider, smax_slider, vmin_slider, vmax_slider;

extern int nb_fx_frame;
extern float pos_robot_x;
extern float pos_robot_y;
extern float angl_robot;






#endif /* PROCESS_HPP_ */
