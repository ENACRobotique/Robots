/*
 * params.hpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#ifndef PARAMS_HPP_
#define PARAMS_HPP_


#define IR_WIDTH (536*2+1)
#define IR_HEIGHT (782)
#define AREA_MIN_FLAT_TRI 14000 //pixels
#define AREA_MAX_FLAT_TRI 18000 //pixels
#define AREA_MIN_DOUBLE_FLAT_TRI 27500 // pixels
#define AREA_MAX_DOUBLE_FLAT_TRI 31500 // pixels
#define LONG_TRI_MIN 132 //mm à z = 25mm
#define LONG_TRI_MAX 148 //mm à z = 25mm
#define LONG_TRI_VERTI_MIN  112//mm à z = 25mm
#define LONG_TRI_VERTI_MAX  132//mm à z = 25mm
#define ANGL_MIN_FLAT_TRI 55
#define ANGL_MAX_FLAT_TRI 65
#define NBR_FEUX 17
#define LONG_MIN_DETECT 30
#define PX2MM 1210./1671.
#define MM2PX 1671./1210.
#define RAD2DEG 180./M_PI
#define DEG2RAD M_PI/180.
#define PERIM_2TRI_CC 540 // Perimeter of two triangle whith the same edges
#define TOL_MAJ_FEUX 10



//######################
//#### Para de simu ####
//######################
#define SAVE false
#define RGL_HSV false



#endif /* PARAMS_HPP_ */
