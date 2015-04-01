/*
 * process.hpp
 *
 *  Created on: 26 mars 2014
 *      Author: yoyo
 */

#ifndef PROCESS_HPP_
#define PROCESS_HPP_

#include "tools.hpp"

// ################## declarations ##################
typedef struct Info_Feu Info_Feu;
//typedef enum Etat_Feu Etat_Feu;
//typedef enum Coul_Feu Coul_Feu;

typedef enum {
	Vertical, Horizontal, Oblique
}Etat_Feu;

typedef enum{
	Rien, Torche, Foyer, Torche_fixe, Feu_R, Feu_J, Autre_Obj
}Obj_Supp;

typedef enum {
	Rouge, Jaune, Autre_Coul
}Coul_Feu;

struct Info_Feu{
	float x_feu;
	float y_feu;
	float theta_feu;
	Etat_Feu etat_feu;
	Coul_Feu coul_feu;
	Obj_Supp obj_supp;

};



void on_trackbar(int, void*);
int process_frame(Mat img_brut);
void affich_Infos_feux(int nb_affich, bool affic_Infos_feux, bool affic_Infos_feux_cap,bool only_fire_detected);
const char* affich_etat_feu(int etat_feu);


extern Info_Feu Infos_feux[NBR_FEUX], Infos_feux_cap[NBR_FEUX];
extern Scalar hsv_min,hsv_max;
/// Global Variables
extern const int h_slider_max, sv_slider_max;
extern int hmin_slider, hmax_slider, smin_slider, smax_slider, vmin_slider, vmax_slider;

extern int nb_fx_frame;
extern float pos_robot_x;
extern float pos_robot_y;
extern float angl_robot;






#endif /* PROCESS_HPP_ */
