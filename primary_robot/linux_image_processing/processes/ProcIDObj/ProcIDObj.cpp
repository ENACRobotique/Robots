/*
 * ProcIDObj.cpp
 *
 *  Created on: Nov 11, 2015
 *      Author: yoyo
 */

#include "ProcIDObj.h"
#include <fstream>
#include <iostream>

ProcIDObj::ProcIDObj(Cam* c, const std::string& objPlgrdFile){
    camList.push_back(c);

    // Load the template objects
    std::ifstream infile(objPlgrdFile);
    if(!infile){
        std::cout << "Can't open file " << objPlgrdFile << std::endl;
        std::exit( -1 );
    }
    std::string line;
    while (getline(infile, line)) {
        std::istringstream s(line);

        string obj;
        string charComment("#");
        std::vector<double> dim;
        std::vector<int> RGB_color;

        s >> obj;

        eObjType objType = ObjTypeMax;
        if(obj.compare(0,string("Parallelepiped").length(), "Parallelepiped") == 0)
            objType = Parallelepiped;
        if(obj.compare("Cylinder") == 0)
            objType = Cylinder;
        if(obj.compare("Cone") == 0)
            objType = Cone;

        std::string w;
        std::size_t posComma;
        switch(objType){
        case Parallelepiped:
            std::cout<<"Parallelepiped found"<<endl;
            obj = obj.substr(string("Parallelepiped").length()+1);

            for(int i=0; i<3; i++){
                posComma = obj.find(',');
                w = obj.substr(0, posComma);
                dim.push_back(stod(w));
                assert(dim.at(i) >= 0 && dim.at(i) <= 1000);
                obj = obj.substr(posComma+1);
            }
            for(int i=0; i<3; i++){
                posComma = obj.find(',');
                w = obj.substr(0, posComma);
                RGB_color.push_back(stod(w));
                assert(RGB_color.at(i) >= 0 && RGB_color.at(i) <= 255);
                obj = obj.substr(posComma+1);
            }

            _objList.push_back(new Play_Obj(Parallelepiped, dim, RGB_color));

            break;
        case Cone:
            std::cout<<"Cone found"<<endl;
            obj = obj.substr(string("Cone").length()+1);

            for(int i=0; i<2; i++){
                posComma = obj.find(',');
                w = obj.substr(0, posComma);
                dim.push_back(stod(w));
                assert(dim.at(i) >= 0 && dim.at(i) <= 1000);
                obj = obj.substr(posComma+1);
            }
            for(int i=0; i<3; i++){
                posComma = obj.find(',');
                w = obj.substr(0, posComma);
                RGB_color.push_back(stod(w));
                assert(RGB_color.at(i) >= 0 && RGB_color.at(i) <= 255);
                obj = obj.substr(posComma+1);
            }

            _objList.push_back(new Play_Obj(Cone, dim, RGB_color));
            _objList.at(1)->print();
            break;
        case Cylinder:
            std::cout<<"Cylinder found"<<endl;
            break;
        case ObjTypeMax:
            break;
        default:
            std::cout<<"Unknown type of object:"<<objType<<std::endl;
        }
    }
    infile.close();

    cout << "Read " << _objList.size() << " template objects from file \"" << objPlgrdFile << "\"" << endl;
}

ProcIDObj::~ProcIDObj(){
}

void ProcIDObj::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU){
    std::cout<<"Process ProcIObj: start"<<std::endl;
    std::cout<<"Process ProcIObj: end"<<std::endl;
}
