/*
 * params.hpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#ifndef PARAMS_HPP_
#define PARAMS_HPP_


//######################
//#### Para of simu ####
//######################
#define PC
#define SAVE false
#define SETTINGS_HSV false
#define SRC_VID 1  // 0 for cam, 1 for yoyo, 2 for ludo, x for someone else



//################################################################
//#### Remarkable values about the elements of the playground ####
//################################################################


//#######################
//#### Para of video ####
//#######################
// Conversions
#define PX2MM 1210./1671.
#define MM2PX 1671./1210.
// Straightened frame
#define IR_WIDTH (536*2+1)
#define IR_HEIGHT (782)


#endif /* PARAMS_HPP_ */
