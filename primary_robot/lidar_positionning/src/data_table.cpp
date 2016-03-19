/*
 * data.cpp
 *
 *  Created on: 20 févr. 2016
 *      Author: fabien
 */
#include "data_table.h"

vector <object> tableObjects = {
		{300,-24,18,"Drapeau Vio1"},	//drapeau violet
		{600,-24,18,"Drapeau Vio2"},	//drapeau violet
		{2400,-24,18,"Drapeau Vert1"},	//drapeau vert
		{2700,-24,18,"Drapeau Vert2"},	//drapeau vert
		{1500,0, 140,"Dune"}	//dune
};



vector <object> balisesViolet = {
		{-62,1000,80,"B vio 1"},
		{3062,-62,80,"B vio 2"},
		{3062,2062,80,"B vio 3"}
};

vector <object> balisesGreen = {
		{3062,1000,80, "B vert 1"},
		{-62,-62,80, "B vert 2"},
		{-62,2062,80, "B vert 3"}
};

vector <object> objs = tableObjects;

void initObjs() {
	objs.insert(objs.end(), balisesGreen.begin(), balisesGreen.end());
}

vector <object> getObjs() {
	return objs;
}

int objDist(object obj1, object obj2){
	return (int)sqrt(pow(obj1.x-obj2.x,2) + pow(obj1.y-obj2.y,2));
}

