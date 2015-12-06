/*
 * Playobj.cpp
 *
 *  Created on: Nov 11, 2015
 *      Author: yoyo
 */

#include "PlayObj.h"

Play_Obj::Play_Obj(eObjType type, vector<double>& dim, vector<int>& RGB_color){
    assert(type == Parallelepiped || type == Cone || type == Cylinder);
    _type = type;
    _dim = dim;
    _RGB_color = RGB_color;
}

Play_Obj::~Play_Obj(){}

void Play_Obj::setConf(AbsPos2D<double> conf){
    _conf = conf;
}

void Play_Obj::print(){
    switch(_type){
    case Parallelepiped:
        cout<<"Parallelepiped :"<<endl;
        break;
    case Cone:
        cout<<"Cone :"<<endl;
        break;
    case Cylinder:
        cout<<"Cylinder :"<<endl;
        break;
    default:
        cout<<"Unknown type of playground object"<<endl;
    }

    cout<<"dim = ";
    for(int i=0; i<(int)_dim.size(); i++){
        cout<< _dim.at(i);
        if(i < (int)_dim.size()-1)
            cout<<", ";
        else
            cout<<endl;
    }

    cout<<"color = ";
    for(int i=0; i<(int)_RGB_color.size(); i++){
        cout<< _RGB_color.at(i);
        if(i < (int)_RGB_color.size()-1)
            cout<<", ";
        else
            cout<<endl;
    }
}
