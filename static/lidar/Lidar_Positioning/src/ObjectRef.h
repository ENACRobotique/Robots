/**
 * Project Untitled
 */


#ifndef _OBJECTREF_H
#define _OBJECTREF_H

#include <string>
#include <stdlib.h>
#include <vector>
using namespace std;


/**
 * \struct ObjectRef ObjectRef.h
 * \brief	Objets de références de la table
 * \details	Ces objets sont fixes et connus à l'avance.
 */
typedef struct ObjectRef {
	int size;		/*!< Taille de l'objet*/
	int posX;		/*!< position en x de l'objet*/
	int posY;		/*!< position en y de l'objet*/
	//const char *name;	/*!< Nom de l'objet*/
	string name;
} ObjectRef;


void initObjects();
void affObjects();
extern vector<ObjectRef*> refObjects;
#endif //_OBJECTREF_H
