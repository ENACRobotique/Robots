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
    if( loadListObj(objPlgrdFile) < 0  ||  (int)_objList.size() <= 0)
        cout << "Could'nt load template objects from file \"" << objPlgrdFile << "\"" << endl;
//    printObjList();

}

ProcIDObj::~ProcIDObj(){
}

void ProcIDObj::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU){
    std::cout<<"Process ProcIObj: start"<<std::endl;
    cout<<"acqList.size() = "<<acqList.size()<<endl;

    for(std::vector<Acq*>::const_iterator itAcq = acqList.begin(); itAcq != acqList.end(); ++itAcq){
        cv::Mat hsv = (*itAcq)->getMat(HSV);
        cv::imwrite("hsv.png", hsv);

        static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
        ProjAcq pAcq = (*itAcq)->projectOnPlane(pl);
        cv::Mat im_pAcq = pAcq.getMat(BGR);

        cv::imwrite("im_pAcq.png", im_pAcq);

        cv::Mat pt_px = (cv::Mat_<float>(3, 1) << 200., 200., 1.);
        cv::Mat pt_cm;
        cout<<"pt_px = "<<pt_px<<endl;
        pt_cm = pAcq.imProj2Plane(pt_px);
        cout<<"pt_cm_R = "<<pt_cm<<endl;
    }


    std::cout<<"Process ProcIObj: end"<<std::endl;
}

int ProcIDObj::loadListObj(const std::string& objPlgrdFile){
    std::ifstream infile(objPlgrdFile);
        if(!infile){
            std::cout << "Can't open file " << objPlgrdFile << std::endl;
            std::exit( -1 );
        }
        std::string line;
        int nbPara =0, nbCone = 0, nbCylinder = 0;
        while (getline(infile, line)) {
            std::istringstream s(line);
            string obj;
            string charComment("#");
            std::vector<double> dim;
            std::vector<int> RGB_color;
            eObjType objType = ObjTypeMax;
            std::string w;
            std::size_t posComma;

            s >> obj;

            if(obj.compare(0, string("#").length(), "#") == 0)
                continue;
            if(obj.compare(0,string("Parallelepiped").length(), "Parallelepiped") == 0)
                objType = Parallelepiped;
            if(obj.compare(0,string("Cylinder").length(), "Cylinder") == 0)
                objType = Cylinder;
            if(obj.compare(0, string("Cone").length(), "Cone") == 0)
                objType = Cone;

            switch(objType){
            case Parallelepiped:
                nbPara++;
                obj = obj.substr(string("Parallelepiped").length()+1);
                dim.clear();
                RGB_color.clear();

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
                nbCone++;
                obj = obj.substr(string("Cone").length()+1);
                dim.clear();
                RGB_color.clear();

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
                break;
            case Cylinder:
                nbCylinder++;
                obj = obj.substr(string("Cylinder").length()+1);
                dim.clear();
                RGB_color.clear();

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

                _objList.push_back(new Play_Obj(Cylinder, dim, RGB_color));
                break;
            case ObjTypeMax:
                break;
            default:
                std::cout<<"Unknown type of object:"<<obj<<std::endl;
                return -1;
            }
        }
        infile.close();

        cout << "Read " << _objList.size() << " template objects from file \"" << objPlgrdFile << "\"" <<
                nbPara << " Parallelepiped(s), "<< nbCylinder <<" Cylinder(s), " << nbCone << " Cone(s)"<<endl;
    return 0;
}

void ProcIDObj::printObjList(){
    cout<<"_____objList loaded_____"<<endl;
    for(int i=0; i<(int)_objList.size(); i++){
        _objList.at(i)->print();
    }
    cout<<"________________________"<<endl;
}
