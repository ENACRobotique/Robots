/*
 * Playobj.cpp
 *
 *  Created on: Nov 11, 2015
 *      Author: yoyo
 */

#include "PlayObj.h"

Play_Obj::Play_Obj(eObjType type, eObjShape shape, vector<float>& dim, eObjCol color){
//    assert(type == sandCube || type == sandCone || type == sandCyl ||
//           type == shellGreen  ||  type == shellViolet  ||  type == shellWhite); // FIXME: enable
//    assert(shape == parallelepiped || shape == cone || shape == cylinder);  // FIXME: enable
    _type = type;
    _shape = shape;
    _dim = dim;
    _col = color;
}

Play_Obj::Play_Obj(Play_Obj* playObj, Pos3D<float>& conf){
    _type = playObj->getType();
    _shape = playObj->getShape();
    _dim = playObj->getDim();
    _col = playObj->getCol();
    _conf = conf;
}

Play_Obj::Play_Obj(eObjType type, Pos3D<float> conf, std::vector<Play_Obj*> _listRefObj){
    std::vector<Play_Obj*>::const_iterator it = _listRefObj.begin();
    _type = type;
    _conf = conf;

    for(it = _listRefObj.begin(); it != _listRefObj.end(); ++it){
        if((*it)->getType() == type){
            _shape = (*it)->getShape();
            _col = (*it)->getCol();
        }
    }
}


Play_Obj::~Play_Obj(){}

void Play_Obj::setConf(Pos3D<float> conf){
    _conf = conf;
}

void Play_Obj::print(bool dispConf){
    switch(_type){
    case sandCube:
        cout<<"sandCube :"<<endl;
        break;
    case sandCone:
        cout<<"sandCone :"<<endl;
        break;
    case sandCyl:
        cout<<"sandCyl :"<<endl;
        break;
    case shellGreen:
        cout<<"shellGreen :"<<endl;
        break;
    case shellViolet:
            cout<<"shellViolet :"<<endl;
            break;
    case shellWhite:
            cout<<"shellWhite :"<<endl;
            break;
    default:
        cout<<"Unknown type of playground object"<<endl;
    }

    switch(_shape){
    case parallelepiped:
        cout<<"\t parallelepiped: ";
        break;
    case cone:
        cout<<"\t cone: ";
        break;
    case cylinder:
        cout<<"\t cylinder: ";
        break;
    case shell_cylinder:
            cout<<"\t shell cylinder: ";
            break;
    default:
        cout<<"Unknown shape of playground object"<<endl;
    }

    cout<<" dim = ";
    for(int i=0; i<(int)_dim.size(); i++){
        cout<< _dim.at(i);
        if(i < (int)_dim.size()-1)
            cout<<", ";
        else
            cout<<endl;
    }

    cout<<"\t color = ";
    switch(_col){
    case yellowDaffodil:
        cout<<"yellowDaffodil\n";
        break;
    case whiteTraffic:
        cout<<"whiteTraffic\n";
        break;
    case greenEmerald:
        cout<<"greenEmerald\n";
        break;
    case violetSignal:
        cout<<"violetSignal\n";
        break;
    default:
        cout<<"Unknown = "<<_col<<endl;
    }

    if(dispConf){
        cout<<"\t Pos3D = ";
        for(int i=0; i<3; i++){
            cout<< _conf.getPt3D()[i];
            if(i < 2)
                cout<<", ";
            else
                cout<<endl;
        }
        cout<<"\t Rxyz = ";
        for(int i=0; i<3; i++){
            cout<< _conf.getRxyz()[i];
            if(i < 2)
                cout<<", ";
            else
                cout<<endl;
        }
    }
}

eObjType Play_Obj::getType() const{
    return _type;
}

eObjShape Play_Obj::getShape()const{
    return _shape;
}

eObjCol Play_Obj::getCol()const{
    return _col;
}

std::vector<float> Play_Obj::getDim() const{
    return _dim;
}

bool Play_Obj::isDimEqual(std::vector<float> dim, float eps){
    if(_dim.size() != dim.size())
        return false;

    for(int i=0; i<(int)_dim.size(); i++){
        if( !(dim[i] < (_dim[i] + eps)  &&  dim[i] > (_dim[i] - eps) ))
            return false;
    }

    return true;
}
