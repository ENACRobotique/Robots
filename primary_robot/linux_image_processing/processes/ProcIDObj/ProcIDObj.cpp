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
#include "tools/Pos3D.h"

#include <opencv2/imgproc/imgproc.hpp>

ProcIDObj::ProcIDObj(Cam* c, const std::string& objPlgrdFile, eVidTypeProc typeProcess){
    cout<<"_______ProcIDOj(): start init__________\n";
    camList.push_back(c);
    typeProc = typeProcess;
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
                    cv::circle(imCtrs, cv::Point(approxCtr[i].x, approxCtr[i].y), 2, cv::Scalar(10, 10, 200), 3);
                    cv::putText(imCtrs, to_string(i), cv::Point(approxCtr[i].x, approxCtr[i].y), 2, 1, cv::Scalar(10, 10, 200), 2);
                    vertexesPl0.push_back(pAcq.imProj2Plane(pt_px));
                    cout<<"\t"<<vertexesPl0[i]<<endl;

                }

                cv::imwrite("imCtrs.png", imCtrs);

                vector<Play_Obj*> ObjsFound = recogObj(*(pAcq.getAcq()), vertexesPl0, (eObjCol)col);
                cout<<"Total objects found: "<<ObjsFound.size()<<endl;
                for(int i=0; i<(int)ObjsFound.size(); i++)
                    ObjsFound[i]->print(true);
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
    int sElt = 10;
    cv::Mat elt_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE,
            cv::Size(sElt, sElt), cv::Point( 0, 0 ) );
    cv::Mat elt_dilate = cv::getStructuringElement( cv::MORPH_ELLIPSE,
            cv::Size(sElt, sElt), cv::Point( 0, 0 ) );

    cv::erode( ret, ret, elt_erode );
    cv::dilate( ret, ret, elt_dilate );
    cv::dilate( ret, ret, elt_dilate );
    cv::erode( ret, ret, elt_erode );
    cv::imwrite("im_yD1.png", ret);

    return ret;
}

void ProcIDObj::compContrs(const cv::Mat m, vector<vector<cv::Point>>& listCtrs) {
    cv::Mat mc = m.clone();
    findContours( mc, listCtrs, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );//CV_CHAIN_APPROX_SIMPLE, CV_CHAIN_APPROX_NONE
}

void ProcIDObj::compApproxCtr(const vector<cv::Point>& ctr, vector<cv::Point>& approxCtr){
    approxPolyDP(cv::Mat(ctr), approxCtr, arcLength(cv::Mat(ctr), true) * 0.02, true);
}

pair<eObjShape, Pos3D<float>> ProcIDObj::recogPuck(const Acq& acq, const vector<cv::Mat>& vertexes){
//    Pos3D<float> posShape;  // Contains the configuration of the shapes found
    pair<eObjShape, Pos3D<float>> shapes;

    double max=70;
    unsigned int min = 0;

    for(unsigned int i = 0; i < vertexes.size(); i++){
        if(cv::norm(vertexes[i], vertexes[(i+1)%vertexes.size()], cv::NORM_L2) > max){
            cout << "Towel detected" << endl;
            return shapes;
        }
        else if(cv::norm(vertexes[i], vertexes[(i+1)%vertexes.size()], cv::NORM_L2) < cv::norm(vertexes[min],vertexes[(min+1)%vertexes.size()], cv::NORM_L2)){
            min = i;
        }
    }

    Pos3D<float> posShape (vertexes[min]);

    if(Vector3D<float>(vertexes[(min + 1)%vertexes.size()]).norm() > Vector3D<float>(vertexes[(min - 1)%vertexes.size()]).norm()){
//        posShape += Pos3D<float>(vertexes[(min - 1)%vertexes.size()]);
    }
    else
    {
//        posShape += Pos3D<float>(vertexes[(min + 1)%vertexes.size()]);
    }
//            shapes = (std::make_pair(parallelepiped, (Pos3D<float>)(posShape*0.5)));

    return shapes;
}

/**
 * @description: Try to identify the shape describe by the contour "ctr".
 *      If the shape is recognized then set the type of object in "t",
 *      otherwise "t" is equal to the possible maximum number of type of object.
 * @note: For now only recognize a quadrilateral
 */
vector<pair<eObjShape, Pos3D<float>>> ProcIDObj::recogShape(const Acq& acq, const vector<cv::Mat>& vertexes){
    vector<Pos3D<float>> shapeFound;  // Contains the configuration of the shapes found
    vector<pair<eObjShape, Pos3D<float>>> shapes;

    if(isNotObject(vertexes)){
        return shapes;
    }

    int s = (int)vertexes.size();
    if(isThereCube(vertexes, acq)  &&  s==6) { // TODO: do not depend of s == 6
        cout<<"s = 6\n";
        shapeFound = recoCubeAside(acq.getCam()->getMatC2R(), vertexes);
        for(int i=0; i<(int)shapeFound.size(); i++){
            shapes.push_back(std::make_pair(parallelepiped, shapeFound[i]));
        }
    }
    else{
        cout<<"Shape not processed yet: s = "<<s<<endl;
    }

    return shapes;
}

bool ProcIDObj::isNotObject(const vector<cv::Mat>& vertexes){
    double min=40, max=150;
    for(unsigned int i = 0; i < vertexes.size(); i++){
        if(cv::norm(vertexes[i], vertexes[(i+1)%vertexes.size()], cv::NORM_L2) < min &&
        (cv::norm(vertexes[i], vertexes[(i-1)%vertexes.size()], cv::NORM_L2) > max || max < cv::norm(vertexes[(i+1)%vertexes.size()], vertexes[(i+2)%vertexes.size()], cv::NORM_L2))){
            cout << "Cleat detected, vertexes : " << i << endl;
            return false;
        }
    }
//    cv::norm(vertexes[ind_pt-1], vertexes[ind_pt+1], cv::NORM_L2);
    return true;
}

bool ProcIDObj::isThereCube(const vector<cv::Mat>& vertexes, const Acq& acq){
    // Find right angle
    float m = 0.2;
    Vector3D<float> vp, vn;
    int s = (int)vertexes.size();
    for(int i=-1; i<=1; i++){ // Just check if a cube is the nearest object to the robot
        vp = Vector3D<float>(vertexes[(s - 1 + i)%s] - vertexes[(s + i)%s]);  // [(s - 1 - i)%s] <=> ptp (previous point from the right angle pt)
        vn = Vector3D<float>(vertexes[(s + i)%s] - vertexes[(s + i + 1)%s]);  // [(s + 1 - i)%s] <=> ptp (next point from the right angle pt)
        float angle = vp.angle(vn);
        cout<<"angle("<<i<<") = "<<angle<<endl;
        if(fabs(angle) < M_PI_2l*(1 + m)  &&  fabs(angle) > M_PI_2l*(1 - m)){
            cout<<"right angle found: vertex "<<i<<"; "<<M_PI_2l*(1 - m)<<" < "<<fabs(angle)<<" < "<<M_PI_2l*(1 + m)<<endl;
//            float lvp = vp.norm(),
//                  lvn = vn.norm();
//            float dimCube = getObjInListRef(sandCube)->getDim()[0]/10;
//            cout<<"lvp = "<<lvp<<", lvn = "<<lvn<<", dimCube = "<<dimCube<< endl;
//
//            Vector3D<float> oCam_R = acq.getCam()->getOrigCamVect3D_R();
//            cout<<"oCam_R = "<<oCam_R<<endl;
//            Vector3D<float> vC2Ptc = Vector3D<float>(vertexes[(s + i)%s]) - oCam_R;
//            cout<<"vC2Ptc = "<<vC2Ptc<<endl;
//            Vector3D<float> vC2Ptn = Vector3D<float>(vertexes[(s + 1 + i)%s]) - oCam_R;
//            Vector3D<float> vC2Ptp = Vector3D<float>(vertexes[(s - 1 + i)%s]) - oCam_R;
//            Vector3D<float> ptcp_R = dimCube/lvp*vC2Ptc + oCam_R;
//            Vector3D<float> ptcn_R = dimCube/lvn*vC2Ptc + oCam_R;
//            cout<<"ptcp_R = "<<ptcp_R<<endl;
//            cout<<"ptcn_R = "<<ptcn_R<<endl;
//            Vector3D<float> ptp_R = lvp/dimCube*vC2Ptp + oCam_R;
//            Vector3D<float> ptn_R = lvn/dimCube*vC2Ptn + oCam_R;
//
//            cout<<"ptcp_R.z = "<<ptcp_R.z()<<", ptcn_R.z = "<<ptcn_R.z()<<", ptp_R.z = "<<ptp_R.z()<<", ptn_R.z = "<<ptn_R.z()<<endl;
            return true;
        }
    }
    return false;
}

bool ProcIDObj::isThereCone(const vector<cv::Mat>& vertexes, const Acq& acq){
    // Find right angle
    float m = 0.2; // +- 18°
    float ref_angle = 0.9; // ~51.5°
    Vector3D<float> vp, vn;
    int s = (int)vertexes.size();
    for(int i=-1; i<=1; i++){ // Just check if a cube is the nearest object to the robot
        vp = Vector3D<float>(vertexes[(s - 1 + i)%s] - vertexes[(s + i)%s]);  // [(s - 1 - i)%s] <=> ptp (previous point from the right angle pt)
        vn = Vector3D<float>(vertexes[(s + i)%s] - vertexes[(s + i + 1)%s]);  // [(s + 1 - i)%s] <=> ptp (next point from the right angle pt)
        float angle = vp.angle(vn);
        cout<<"angle("<<i<<") = "<<angle<<endl;
        if(fabs(angle) < M_PI_2l*(ref_angle + m)  &&  fabs(angle) > M_PI_2l*(ref_angle - m)){
            cout<<"cone top angle found: vertex "<<i<<"; "<<M_PI_2l*(1 - m)<<" < "<<fabs(angle)<<" < "<<M_PI_2l*(1 + m)<<endl;
            return true;
        }
    }
    return false;
}

vector<Play_Obj*>ProcIDObj::recogObj(const Acq& acq, vector<cv::Mat>& vertexes, eObjCol col){
    vector<Play_Obj*> objectsFound;
    pair<eObjShape, Pos3D<float>> vShape;
    vector<pair<eObjShape, Pos3D<float>>> vShapes;
    // Find the nearest pt "pt0" to the robot and arrange "vertexes" such that vertexes[0] is pt0. (trigonometric direction)
    int indexPt0 = getNearestPtTo(vertexes, (cv::Mat_<float>(3,1)<< 0.,0.,0.));
    translateValVector(vertexes, indexPt0);

    if(col != yellowDaffodil){
       vShape = recogPuck(acq, vertexes);
       objectsFound.push_back(new Play_Obj ((getSameInListRefObj(col, vShape.first)[0]),vShape.second));
    }
    else
    {
        vShapes = recogShape(acq, vertexes);
        for(int i=0; i<(int)vShapes.size(); i++){
            vector<Play_Obj*> vObj = getSameInListRefObj(col, vShapes[i].first);
            if((int)vObj.size() == 1)
                objectsFound.push_back(new Play_Obj((getSameInListRefObj(col, vShapes[i].first)[0]),vShapes[i].second));
            else
                cout<<"recoObj(): Can't recognize object: too many solution ("<<vObj.size()<<")\n";
        }
    }

    return objectsFound;
}

eObjType ProcIDObj::recogObjType(eObjCol col, eObjShape shape){
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

/**
 * Description: Detect and compute a vector of configuration (x, y, theta) for each cubes in the global shape
 * Note: Consider that the whole shape contains sand cubes with the same arbitrary orientation (angle between the y axis of the robot and the face of the cube between points 0 and 1)
 */
vector<Pos3D<float>> ProcIDObj::recoCubeAside(const cv::Mat C2R, const vector<cv::Mat>& vertexes){
    vector<Pos3D<float>> cubesFound;
    const eObjType objType = sandCube;
    std::cout << "recoCubeAside(): Start shape analyse..." << endl;

    // Compute the number of objects following the two direction in the plane of the table
    int nbCubeH05 = compNbIdenticObjH(objType, vertexes[0], vertexes[5]);
    int nbCubeH01 = compNbIdenticObjH(objType, vertexes[0], vertexes[1]);

    // Compute the number of object following the vertical direction
    cv::Mat ptCam_R = C2R(Rect(3,0,1,3));
    int nbCubeV = compNbIdenticObjV(objType, ptCam_R, vertexes[1], vertexes[2]);
    cout<<"nbCubeH05 = "<<nbCubeH05<<", nbCubeH01 = "<<nbCubeH01<<", nbCubeV = "<<nbCubeV<<endl;

    // Compute the direction of the faces 0-1 and 0-5
    Vector3D<float> dirFace01 = Vector3D<float>(vertexes[1] - vertexes[0]).normalize();
    dirFace01 = dirFace01.rotate(-M_PI_2l, Vector3D<float>(0., 0., 0.), Vector3D<float>(0., 0., 1.));

    const Vector3D<float> dirFace05 = Vector3D<float>(vertexes[5] - vertexes[0]).normalize().rotate(M_PI_2, Vector3D<float>(0., 0., 0.), Vector3D<float>(0., 0., 1.));
    const float theta = Vector3D<float>(0., 1.,0.).angle(dirFace01);  // Arbitrary direction of the face 0-1
    cout<<"dirFace01 = "<<dirFace01<<endl;
    cout<<"dirFace05 = "<<dirFace05<<endl;
    cout<<"theta = "<<theta*180/M_PIl<<" deg"<<endl;

    // Create the list of configuration of the centers of each cube
    float dim = getObjInListRef(sandCube)->getDim()[0]/10.;
    Vector3D<float> c0 = Vector3D<float>(vertexes[0]) - Vector3D<float>(dim/2*(dirFace01 + dirFace05 - Vector3D<float>::zAxis)).toCv();
    cout<<"c0 = "<<c0<<endl;
    for(int h05=0; h05<nbCubeH05; h05++){
        for(int h01=0; h01<nbCubeH05; h01++){
            for(int v=0; v<nbCubeV; v++){
                Vector3D<float> vect = Vector3D<float>::zAxis*v - dirFace01*h01 - dirFace05*h05;
                cubesFound.push_back(Pos3D<float>(c0 + dim*(vect), 0., 0., theta));
            }
        }
    }

    return cubesFound;
}

Play_Obj* ProcIDObj::getObjInListRef(eObjType objType){
    for(int i=0; i<(int)_listRefObj.size(); i++){
        if(_listRefObj[i]->getType() == objType)
            return _listRefObj[i];
    }

    // Unknown type
    cout<<"getObjInListRef(): "<< objType <<"unknown type of obj\n";
    return nullptr;
}

int ProcIDObj::compNbIdenticObjH(const eObjType objType, const cv::Mat& pt1, const cv::Mat& pt2, const float err){
    // TODO: use err
    float dim;
    switch(objType){
    case sandCube:
        dim = getObjInListRef(sandCube)->getDim()[0];
        break;
    default:
        cout<<"compNbIdenticObjH(): objType not processed yet\n";
    }
    float nbCubeH = 0;
    try {
        cout<<"dist = "<<cv::norm(pt1, pt2, cv::NORM_L2)*10<<endl;
        nbCubeH = (cv::norm(pt1, pt2, cv::NORM_L2)*10/dim + 0.5);
    } catch (Exception& e) {
        cout<<e.what()<<endl;
    }

    return (int)nbCubeH;
}

Play_Obj* ProcIDObj::findCone(const cv::Mat& ptCam_R, const vector<cv::Mat>& vertexes){
    Play_Obj* cone(getObjInListRef(sandCone));
//    const float theta = Vector3D<float>(0., 1.,0.).angle(dirFace01);  // Arbitrary direction of the face 0-1
    float angle;
    for(unsigned int i = 0; i < vertexes.size(); i++){
        angle = Vector3D<float>(vertexes[(i+1)%vertexes.size()]-vertexes[i]).angle(Vector3D<float>(vertexes[(i-1)%vertexes.size()]-vertexes[i]));
        if (angle < (60 * 1.1)*180/M_PI && angle > (60 * 0.9)*180/M_PI){
            cone = findConeCarac(ptCam_R, vertexes, i);
            // TODO : specific part if cone down
        }
    }
    return cone;
}

Play_Obj* ProcIDObj::findConeCarac(const cv::Mat& ptCam_R, const vector<cv::Mat>& vertexes, const int ind_pt){
    Pos3D<float> pos();
    Play_Obj* cone_found(getObjInListRef(sandCone));

    float coradius, cylhigh, factDir;

    // Barycentre deux points autour sommet
    cv::Mat ptTable = vertexes[ind_pt];// ajouter sommet
    cv::Mat vDir = cv::Mat(ptCam_R - ptTable);

    cylhigh = getObjInListRef(sandCyl)->getDim()[0];
    coradius = getObjInListRef(sandCone)->getDim()[1];

    factDir = (2*coradius)/cv::norm(vertexes[ind_pt-1], vertexes[ind_pt+1], cv::NORM_L2);

    cv::Mat ptBase(ptCam_R + factDir * vDir);
    cout<< "Old cone : " << ptBase;
    ptBase.at<float>(2) = cylhigh*roundf(ptBase.at<float>(2)/cylhigh);
    // Recalculer hauteur cone trouvé.
    return cone_found;
}

int ProcIDObj::compNbIdenticObjV(const eObjType objType, const cv::Mat& ptCam_R, const cv::Mat& ptTable, const cv::Mat& ptProj, const float err){
    // TODO: use err
    float dPRoj_R = cv::norm(ptProj, cv::NORM_L2);
    float dTable_R = cv::norm(ptTable, cv::NORM_L2);
    float z = ptCam_R.at<float>(2)*(dPRoj_R - dTable_R)/(dPRoj_R - ptCam_R.at<float>(1))*10; // From mm to cm

    float dim;
    float nbCubeV = 0;
    switch(objType){
    case sandCube:
        dim = getObjInListRef(sandCube)->getDim()[0];
        break;
    default:
        cout<<"compNbIdenticObjH(): objType not processed yet\n";
    }

    try {
        nbCubeV = (z/dim + 0.5);
    } catch (Exception& e) {
        cout<<e.what()<<endl;
    }

    return (int)nbCubeV;
}
