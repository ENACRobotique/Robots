/*
 * ProcIDObj.cpp
 *
 *  Created on: Nov 11, 2015
 *      Author: yoyo
 */

#include "ProcIDObj.h"
#include <fstream>
#include <iostream>
#include "Vector3D.h"

#include <opencv2/imgproc/imgproc.hpp>

ProcIDObj::ProcIDObj(Cam* c, const std::string& objPlgrdFile){
    cout<<"_______ProcIDOj(): start init__________\n";
    camList.push_back(c);
    _stateCalib = notDone;

    // Load the template objects
    if( loadListObj(objPlgrdFile) < 0  ||  (int)_listRefObj.size() <= 0){
        cout << "Could'nt load template objects from file \"" << objPlgrdFile << "\"" << endl;
        exit(0);
    }
    printObjList();

    cout<<"_______ProcIDOj(): end init__________\n";
}

ProcIDObj::~ProcIDObj(){
}

void ProcIDObj::CalibHSV(const std::vector<Acq*>& acqList){
    cout<<"_____ CalibHSV(): START _____\n";
    map<int, vector<pair<int, int>>> listCalibArea;
    loadCalibCam("../2016/listCalibCams.csv", listCalibArea);

    for(std::vector<Acq*>::const_iterator itAcq = acqList.begin(); itAcq != acqList.end(); ++itAcq){
        cv::Mat hsv = (*itAcq)->getMat(HSV);
        const int idCam = (*itAcq)->getCam()->getIdCam();
        cout<<"IdCam = "<<idCam<<endl;
        if(listCalibArea.find(idCam) == listCalibArea.end()){
            cout<<"Skip calib cam n°"<<idCam<<endl;
            continue;
        }

        mapCol_T::iterator it = _listAcqColObj[idCam].begin();
        cout<<"_listAcqColObj[idCam].size() = "<<_listAcqColObj[idCam].size()<<endl;
        for(; it!=_listAcqColObj[idCam].end(); ++it){  // For each color
            eObjCol col = static_cast<eObjCol>(it->first);
            cout<<"col = "<<(int)col<<endl;
            cv::Mat im_bin = getBinaryImage(hsv, col, idCam);
            cv::imwrite("im_binCalib.png", im_bin);
            std::vector<std::vector<cv::Point> > listCtrs;
            compContrs(im_bin, listCtrs);
            cout<<"nbContour = "<<listCtrs.size()<<endl;
            float area = cv::contourArea(listCtrs[0]);  // FIXME: only one contour
            cout<<"area = "<<area<<" px²"<<endl;
            // TODO: Tuning of HSV
        }
    }

    _stateCalib = fail;
    cout<<"_____ CalibHSV(): END _____\n";
}

int ProcIDObj::loadCalibCam(const std::string& calibCamFile, map<int, vector<pair<int, int>>>& listCalibArea){
    std::ifstream infile(calibCamFile);
    if(!infile){
        std::cout << "Can't open file " << calibCamFile << std::endl;
        std::exit( -1 );
    }
    std::string line;
    while (getline(infile, line)) {
        std::istringstream s(line);
        string obj;
        eColObj col;
        std::string w;
        std::size_t posComma;
        vector<pair<int, int>> listColArea;

        s >> obj;

        if(obj.compare(0, string("#").length(), "#") == 0 || obj.length() == 0)
            continue;

        posComma = obj.find(',');
        w = obj.substr(0, posComma);
        int idCam = stoi(w);
        assert(idCam >= 0 && idCam <= 5);
        obj = obj.substr(posComma + 1);
        while(obj.length() > 0){
            cout<<"-----"<<endl;
            posComma = obj.find(',');
            w = obj.substr(0, posComma);
            col = static_cast<eColObj>(stoi(w));
            obj = obj.substr(posComma + 1);
            posComma = obj.find(',');
            int area;
            if((int)posComma == -1){
                area = stoi(obj);
                obj.clear();
            }
            else{
                w = obj.substr(0, posComma);
                area = stoi(w);
                obj = obj.substr(posComma + 1);
            }
            listColArea.push_back(make_pair((int)col, area));
        }
        listCalibArea[idCam] = listColArea;
        cout<<"cam: "<<idCam<<", (col, area) = (" << listCalibArea[idCam][0].first<< ", " <<
                listCalibArea[idCam][0].second << ")" << endl;
    }
    infile.close();

    cout << "Load " << listCalibArea.size() << " calibrations for cameras from \"" << calibCamFile <<endl;

    return 0;
}

void ProcIDObj::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU){
    std::cout<<"___________Process ProcIObj: start__________"<<std::endl;
    cout<<"acqList.size() = "<<acqList.size()<<endl;
    _listObj.clear();

//    if(_stateCalib == notDone  ||  _stateCalib == fail){
//        CalibHSV(acqList);
//        return;
//    }

    for(std::vector<Acq*>::const_iterator itAcq = acqList.begin(); itAcq != acqList.end(); ++itAcq){
        int idCam = (*itAcq)->getCam()->getIdCam();
        cv::Mat hsv = (*itAcq)->getMat(HSV);
        cv::imwrite("hsv.png", hsv);

        cv::Mat bgr = (*itAcq)->getMat(BGR);
        cv::imwrite("bgr.png", bgr);

        static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
        ProjAcq pAcq = (*itAcq)->projectOnPlane(pl);
        cv::Mat im_pAcq = pAcq.getMat(HSV);  // FIXME: Initialize before process for game

        cv::imwrite("im_pAcq.png", im_pAcq);

        // Process the image following each color filter (HSV)
        for(int colInt=0; colInt<1/*(int)objColMax*/; colInt++){
            cout<<"Process color: col = "<<colInt<<endl;

            eObjCol col = static_cast<eObjCol>(colInt);
            cv::Mat im_bin = getBinaryImage(im_pAcq, col, idCam);
            cv::imwrite("im_bin.png", im_bin);
            cout<<"M1\n";

            // Extract the contours
            std::vector<std::vector<cv::Point> > listCtrs, listApproxCtrs;
            std::vector<cv::Point> approxCtr;
            cv::Mat imCtrs = im_pAcq.clone();
            cv::Scalar col_ctr;
            compContrs(im_bin, listCtrs);
            cout << "Nb_countours = "<< listCtrs.size() <<endl;

            // Process each contour
            for (size_t idx = 0; idx < listCtrs.size(); idx++) { // for each contour
                cout << "Countours #" << idx << "/ " << listCtrs.size() - 1<<endl;

                col_ctr = cv::Scalar((idx*30)%256, (idx*30)%256, (idx*30)%256);
                cv::drawContours(imCtrs, listCtrs, idx, col_ctr,4);
                cv::imwrite("imCtrs.png", imCtrs);

                // Approximate  the contour
                compApproxCtr(listCtrs[idx], approxCtr);
                col_ctr = cv::Scalar((idx*60)%256, (idx*60)%256, (idx*60)%256);
                listApproxCtrs.push_back(approxCtr);
                cout<<"nbVertCtr = "<<listCtrs[idx].size()<<", nbVertApproxCtr = "<<listApproxCtrs[idx].size()<<endl;
                cv::drawContours(im_pAcq, listApproxCtrs, idx, col_ctr,4);
                cv::imwrite("imCtr1_pAcq.png", im_pAcq);
                cv::imshow("HSV", im_pAcq);

                // Process each approximated contour
                int m = (int)approxCtr.size();
                vector<cv::Mat> vertexesPl0;
                for(int i=0; i<m; i++){
                    cout<<"____ vert"<<i<<" = "<<approxCtr[i]<<endl;
                    cv::Mat pt_px = (cv::Mat_<float>(3, 1) << approxCtr[i].x, approxCtr[i].y, 1.);
                    cv::circle(imCtrs, cv::Point(approxCtr[i].x, approxCtr[i].y), 2, cv::Scalar(10, 10, 200), 2);
                    vertexesPl0.push_back(pAcq.imProj2Plane(pt_px));
                    cout<<"\t"<<vertexesPl0[i]<<endl;
                }

                cv::imwrite("imCtrs.png", imCtrs);

                recogObj(vertexesPl0, (eObjCol)col);
            }
        }
    }


    std::cout<<"_________Process ProcIObj: end____________"<<std::endl;
}

int ProcIDObj::loadListObj(const std::string& objPlgrdFile){
    // TODO: process color from the file
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
        std::vector<float> dim;
        eObjType objType = objTypeMax;
        eColObj ObjCol;
        std::string w;

        s >> obj;

        if(obj.compare(0, string("#").length(), "#") == 0)
            continue;
        if(obj.compare(0,string("sandCube").length(), "sandCube") == 0)
            objType = sandCube;
        if(obj.compare(0,string("sandCone").length(), "sandCone") == 0)
            objType = sandCone;
        if(obj.compare(0, string("sandCyl").length(), "sandCyl") == 0)
            objType = sandCyl;
        if(obj.compare(0, string("shellGreen").length(), "shellGreen") == 0)
            objType = shellGreen;
        if(obj.compare(0, string("shellViolet").length(), "shellViolet") == 0)
            objType = shellViolet;
        if(obj.compare(0, string("shellWhite").length(), "shellWhite") == 0)
            objType = shellWhite;

        switch(objType){
        case sandCube:
            nbPara++;
            obj = obj.substr(string("sandCube").length()+1);
            setDim(dim, obj, 3);
            setColors(ObjCol, obj, 0);

            _listRefObj.push_back(new Play_Obj(sandCube, parallelepiped, dim, ObjCol));
            break;
        case sandCone:
            nbCone++;
            obj = obj.substr(string("sandCone").length()+1);
            setDim(dim, obj, 2);
            setColors(ObjCol, obj, 0);

            _listRefObj.push_back(new Play_Obj(sandCone, cone, dim, ObjCol));
            break;
        case sandCyl:
            nbCylinder++;
            obj = obj.substr(string("sandCyl").length()+1);

            setDim(dim, obj, 2);
            setColors(ObjCol, obj, 0);

            _listRefObj.push_back(new Play_Obj(sandCyl, cylinder, dim, ObjCol));
            break;
        case shellGreen:
            nbCylinder++;
            obj = obj.substr(string("shellGreen").length()+1);

            setDim(dim, obj, 2);
            setColors(ObjCol, obj, 0);

            _listRefObj.push_back(new Play_Obj(shellGreen, cylinder, dim, ObjCol));
            break;
        case shellViolet:
            nbCylinder++;
            obj = obj.substr(string("shellViolet").length()+1);

            setDim(dim, obj, 2);
            setColors(ObjCol, obj, 0);

            _listRefObj.push_back(new Play_Obj(shellViolet, cylinder, dim, ObjCol));
            break;
        case shellWhite:
            nbCylinder++;
            obj = obj.substr(string("shellWhite").length()+1);

            setDim(dim, obj, 2);
            setColors(ObjCol, obj, 0);

            _listRefObj.push_back(new Play_Obj(shellWhite, cylinder, dim, ObjCol));
            break;
        case objTypeMax:
            break;
        default:
            std::cout<<"Unknown type of object:"<<obj<<std::endl;
            return -1;
        }
    }
    infile.close();

    cout << "Read " << _listRefObj.size() << " template objects from file \"" << objPlgrdFile << "\"" <<
            nbPara << " Parallelepiped(s), "<< nbCylinder <<" Cylinder(s), " << nbCone << " Cone(s)"<<endl;

    return 0;
}

void ProcIDObj::setColors(eObjCol& c, string& s, const int idCam){
    std::string w;
    std::size_t posComma;

    // Get type of color
    posComma = s.find(',');
    w = s.substr(0, posComma);
    c = (eObjCol)stoi(w);
    assert(c >= 0 && c <= objColMax);
    s = s.substr(posComma+1);

    // Get the value of the color
    // TODO: Generalize: take into account BGR not only HSV
    cv::Scalar hsv_min, hsv_max;
    for(int i=0; i<3; i++){  // To set hsv_min values
        posComma = s.find(',');
        w = s.substr(0, posComma);
        hsv_min.val[i] = stod(w);
        assert(hsv_min[i] >= 0 && hsv_min[i] <= 256);
        s = s.substr(posComma+1);
    }
    for(int i=0; i<3; i++){  // To set hsv_max values
        posComma = s.find(',');
        w = s.substr(0, posComma);
        hsv_max.val[i] = stod(w);
        assert(hsv_max[i] >= 0 && hsv_max[i] <= 256);
        s = s.substr(posComma+1);
    }

    mapCol_T m;
    if(_listAcqColObj.find(idCam) == _listAcqColObj.end()){ // new acq to add in _listAcqColObj
        m[c] = make_pair(hsv_min, hsv_max);
        _listAcqColObj[idCam] = m;
    }
    else{
        if(_listAcqColObj[idCam].find(c) == _listAcqColObj[idCam].end() ){
            _listAcqColObj[idCam][c] = make_pair(hsv_min, hsv_max);
        }
    }
}

void ProcIDObj::setDim(std::vector<float>& dim, string& s, int n){
    std::string w;
    std::size_t posComma;
    dim.clear();
    for(int i=0; i<n; i++){
        posComma = s.find(',');
        w = s.substr(0, posComma);
        dim.push_back(stod(w));
        assert(dim.at(i) >= 0 && dim.at(i) <= 1000);
        s = s.substr(posComma+1);
    }
}

void ProcIDObj::printObjList(){
    cout<<"_____objList loaded_____"<<endl;
    for(int i=0; i<(int)_listRefObj.size(); i++){
        _listRefObj.at(i)->print();
    }
    cout<<"________________________"<<endl;
}


//std::pair<cv::Scalar, cv::Scalar> ProcIDObj::getHSVForCam(const int idCam, const eObjCol col){
//    std::pair<cv::Scalar, cv::Scalar> ret = _listAcqColObj[idCam][col];
//
//    mapsMapCol_T::iterator it_p = _listAcqColObj.begin();
//    for(; it_p != _listAcqColObj.end(); ++it_p){
//        if((*it_p).first == idCam){
//            mapCol_T::iterator it_t = (*it_p).begin();
//            for(; it_t != (*it_p).second.end(); ++it_t){
//                if(get<0>(*it_t) == col){
//                    ret.push_back(get<1>(*it_t));
//                    ret.push_back(get<2>(*it_t));
//                }
//            }
//
//            break;
//        }
//    }
//    if(!ret.size()){
//        cout<<"Error in ProcIDObj::getHSVForCam(): HSVs not found for: idCam = "<<idCam<<", col = "<<col<<endl;
//    }
//
//    return ret;
//}

cv::Mat ProcIDObj::getBinaryImage(cv::Mat m, eObjCol col, const int idCam){
    cv::Mat ret;
    cv::Scalar hsv_min = cv::Scalar(_listAcqColObj[idCam][col].first);
    cv::Scalar hsv_max = cv::Scalar(_listAcqColObj[idCam][col].second);
    cout<<"hsv_min = "<<hsv_min<<endl<<"hsv_max = "<<hsv_max<<endl;
    cv::inRange(m, hsv_min, hsv_max, ret);
    cv::imwrite("im_pAcq.png", m);
    cv::imwrite("im_yD0.png", ret);

    // Remove small smudges
    int sElt = 8;
    cv::Mat elt_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size( sElt, sElt ), cv::Point( 0, 0 ) );
    cv::Mat elt_dilate = cv::getStructuringElement( cv::MORPH_ELLIPSE,
            cv::Size( sElt, sElt ), cv::Point( 0, 0 ) );

    cv::erode( ret, ret, elt_erode );
    cv::dilate( ret, ret, elt_dilate );
    cv::imwrite("im_yD1.png", ret);

    return ret;
}

void ProcIDObj::compContrs(const cv::Mat m, vector<vector<cv::Point>>& listCtrs) {
    cv::Mat mc = m.clone();
    findContours( mc, listCtrs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );//CV_CHAIN_APPROX_SIMPLE, CV_CHAIN_APPROX_NONE
}

void ProcIDObj::compApproxCtr(const vector<cv::Point>& ctr, vector<cv::Point>& approxCtr){
    approxPolyDP(cv::Mat(ctr), approxCtr, arcLength(cv::Mat(ctr), true) * 0.01, true);
}

/**
 * @description: Try to identify the shape describe by the contour "ctr".
 *      If the shape is recognized then set the type of object in "t",
 *      otherwise "t" is equal to the possible maximum number of type of object.
 * @note: For now only recognize a quadrilateral
 */
eObjShape ProcIDObj::recogShape(const vector<cv::Mat>& vertexes, vector<Vector3D<float>>& edges){
    std::vector<Play_Obj*> ObjectFound;
    eObjShape t = objShapeMax;
    edges.clear();
    int s = (int)vertexes.size();

    if (s == 6) {
        ObjectFound = recoCubeAside(vertexes);
    }

    return t;
}

Play_Obj *ProcIDObj::recogObj(vector<cv::Mat>& vertexes, eObjCol col){
    vector<Vector3D<float>> edges;

    eObjShape shape = recogShape(vertexes, edges);

    eObjType objType = recogObjType(vertexes, col, shape);


    std::vector<float> dim;
    // TODO: Add functions to recognize dim

    return new Play_Obj(objType, shape, dim, col);
}

eObjType ProcIDObj::recogObjType(vector<cv::Mat>& vertexes, eObjCol col, eObjShape shape){
    eObjType type = objTypeMax;
    vector<Play_Obj*> listObj(getSameInListRefObj(col, shape));

    cout<<"recoObj(3p): listObj.size() = "<<listObj.size()<<endl;
    for(int i=0; i<(int)listObj.size(); i++){
        listObj[i]->print();
    }

    if(listObj.size() == 1)
        type  = listObj[0]->getType();
    return type;
}

vector<float> ProcIDObj::getPosOfObj(const eObjType t, const vector<cv::Mat>& vertexes, const vector<Vector3D<float>>& edges){
    vector<float> pos;
    switch(t){
    case sandCube:

        break;
    case sandCone:

        break;
    case sandCyl:

        break;
    default:
        cout<<"getPosOfShape(): Unknown type of playground object = "<<t<<endl;
    }

    return pos;
}

vector<Play_Obj*> ProcIDObj::getSameInListRefObj(eObjCol col, eObjShape shape,
        vector<float> dim, eObjType type, float epsDim) const{
    std::vector<Play_Obj*> listObj;
    std::vector<Play_Obj*>::const_iterator it = this->_listRefObj.begin();

    if(type != objTypeMax){
        for(; it != _listRefObj.end(); ++it){
            if((*it)->getType() == type)
                listObj.push_back(*it);
                return listObj;  // Because it is unique in "_listRefObj"
        }
    }

    if(dim.size() != 0){
        cout<<"dim\n";
        for(it = _listRefObj.begin(); it != _listRefObj.end(); ++it){
            if((*it)->isDimEqual(dim, epsDim))
                listObj.push_back(*it);
        }
    }

    if(shape != objShapeMax){
        std::vector<Play_Obj*> listObjTemp;
        if((int)listObj.size() != 0)
            listObjTemp = listObj;
        else
            listObjTemp = _listRefObj;
        listObj.clear();
        for(it = listObjTemp.begin(); it != listObjTemp.end(); ++it){
            if((*it)->getShape() == shape)
                listObj.push_back(*it);
        }
    }

    if(col != objColMax){
        std::vector<Play_Obj*> listObjTemp(listObj);
        listObj.clear();
        for(it = listObjTemp.begin(); it != listObjTemp.end(); ++it){
            if((*it)->getCol() == col)
                listObj.push_back(*it);
        }
    }

    return listObj;
}

vector<Play_Obj*> ProcIDObj::recoCubeAside(const vector<cv::Mat>& vertexes){

    vector<Play_Obj*> Objets_found;
    std::cout << "Start shape analyse..." << endl;
    cv::vector<cv::Mat> mxt;
    cv::Mat pt_temp, barycentre = vertexes[0];
    float temp = 0;


    for(int i=1;i < (int)vertexes.size();i++){
        cout << "Point : " << i << ", x:" << vertexes[i].at<float>(0) << ", y:" << vertexes[i].at<float>(1) << endl;
        barycentre += vertexes[i];
    }
    cout<< "barycentre : " << barycentre << endl;
    barycentre /= 6;
    cv::Mat ref = barycentre;
    ref.at<float>(0)-=1;
    cout<< "ref : " << ref << endl;
//    Vector3D<float>(cv::Mat(vertexes[i] - vertexes[(i+1)%s]))
//    Vector3D<float>drt_ref=(cv::Mat(barycentre - ref));
    temp = Vector3D<float>(ref - barycentre).angle(Vector3D<float>(vertexes[0] - barycentre));
    cout << "droite" << Vector3D<float>(ref) << endl;
    cout << "droite" << Vector3D<float>(vertexes[0]) - Vector3D<float>(barycentre) << endl;
    cout << "droite" << (vertexes[0]) << endl;
    cout << "droite" << (barycentre) << endl;
    cout << "droite" << temp << endl;
    for(int i=0;i < (int)vertexes.size();i++){

    }

//
    for(int i = 0;i < (int)vertexes.size();i++){
        if (vertexes[i].at<float>(1)>= temp) {
            vertexes[2] = vertexes[1];
            vertexes[1] = vertexes[0];
            vertexes[0] = vertexes[i];
            cout << i << "haut" << endl;
        }
    }
//        else{
//            vertexes[5] = vertexes[4];
//            vertexes[4] = vertexes[3];
//            vertexes[3] = vertexes[i];
//            cout << i << "bas" << endl;
//        }
//
//    }

    for(int i = 0; i < (int)vertexes.size(); i++){
        cout << "Point : " << i << ", x:" << vertexes[i].at<float>(0) << ", y:" << vertexes[i].at<float>(1) << endl;
    }



//    nb_deep = roundf(d56/DIM);
//    nb_horiz = roundf(d45/DIM);
//    nb_vert = roundf(H0*d34/(d3*DIM));


    /*
    float alpha = angle(ref / pt4-pt5)
    float X = DIM*(cos(alpha) + cos(alpha + 90);
    float Y = DIM*(sin(alpha) + sin(alpha + 90);

    for(int i = 0; i < nb_H; i++){
        for(int j = 0; j < nb_D; j++){
            for(int k = 0; k < nb_V; k++){
                Objects_found(i*nb_D*nb_V + j*nb_V + k).at<float>(0) = x0 + X(i + 1/2);
                Objects_found(i*nb_D*nb_V + j*nb_V + k).at<float>(1) = y0 + Y(j + 1/2);
                Objects_found(i*nb_D*nb_V + j*nb_V + k).at<float>(2) = DIM(k + 1/2);
            }
        }
    }

*/
    return Objets_found;
}
