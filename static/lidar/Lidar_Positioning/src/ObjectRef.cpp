#include "ObjectRef.h"
#include "ostream"
#include "stdio.h"
using namespace std;
#define NB_OBJS 11

//origine : coin dune / départ vert
ObjectRef objects_vector[] = {
		{18,2400,2000,"Drapeau Vert1"},	//drapeau vert
		{18,2700,2000,"Drapeau Vert2"},	//drapeau vert
		{18,300,2000,"Drapeau Vio1"},	//drapeau violet
		{18,600,2000,"Drapeau Vio2"},	//drapeau violet
		{190,1500,2000,"Dune"},			//dune
		{80,3062,2062,"B vio 1"},			//balise violette 1 (origine)
		{80,0,1000,"B vio 2"},		//balise violette 2 (zone départ violet)
		{80,3062,0,"B vio 3"},		//balise violette 3 (rocher coté vert)
		{80,0,2062, "B vert 1"},		//balise verte 1 (cabanes violettes)
		{80,3062,1000, "B vert 2"},		//balise verte 2 (zone départ vert)
		{80,0,0, "B vert 3"}		//balise verte 3 (rocher coté violet)
};

vector<ObjectRef*> refObjects;

void initObjects() {
	for(int i=0;i<NB_OBJS;i++) {
		refObjects.push_back(&objects_vector[i]);
		//printf("obj : %s \n",objects_vector[i].name);
	}
}

void affObjects() {
	printf("Début affichage :\n");
	for(ObjectRef *obj : refObjects) {
		//printf("aff obj : %s \n",obj->name);
	}
}
