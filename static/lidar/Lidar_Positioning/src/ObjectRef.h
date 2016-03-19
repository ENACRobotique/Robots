/**
 * Project Untitled
 */


#ifndef _OBJECTREF_H
#define _OBJECTREF_H

#include <string>
#include <stdlib.h>
using namespace std;

class ObjectRef {
public: 
    
    int getSize();
    
    int getX();
    
    int getY();
    
    string getName();
private: 
    int size;
    int posX;
    int posY;
    string name;
};

#endif //_OBJECTREF_H
