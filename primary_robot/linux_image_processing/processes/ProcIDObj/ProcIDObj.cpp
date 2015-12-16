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

    // Load the template objects
    if( loadListObj(objPlgrdFile) < 0  ||  (int)_listRefObj.size() <= 0){
        cout << "Could'nt load template objects from file \"" << objPlgrdFile << "\"" << endl;
        exit(0);
    }
//    printObjList();

    cout<<"_______ProcIDOj(): end init__________\n";
}

ProcIDObj::~ProcIDObj(){
}

void ProcIDObj::process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU){
    std::cout<<"___________Process ProcIObj: start__________"<<std::endl;
    cout<<"acqList.size() = "<<acqList.size()<<endl;
    _listObj.clear();

    for(std::vector<Acq*>::const_iterator itAcq = acqList.begin(); itAcq != acqList.end(); ++itAcq){
        cv::Mat hsv = (*itAcq)->getMat(HSV);
        cv::imwrite("hsv.png", hsv);

        static Plane3D<float> pl( { 0, 0, 0 }, { 0, 0, 1 }); // build a plane with a point and a normal
        ProjAcq pAcq = (*itAcq)->projectOnPlane(pl);
        cv::Mat im_pAcq = pAcq.getMat(HSV);

        cv::imwrite("im_pAcq.png", im_pAcq);

        // Test projection to plane
//        cv::Mat pt_px = (cv::Mat_<float>(3, 1) << 200., 200., 1.);
//        cv::Mat pt_cm;
//        cout<<"pt_px = "<<pt_px<<endl;
//        pt_cm = pAcq.imProj2Plane(pt_px);
//        cout<<"pt_cm_R = "<<pt_cm<<endl;

        // Process the image following each color filter (HSV)
        for(int col=0; col<1/*(int)objColMax*/; col++){
            cout<<"Process color: col = "<<col<<endl;

            cv::Mat im_bin = getBinaryImage(im_pAcq, col);
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
                cout << "Countours #" << idx << "/ " << listCtrs.size() <<endl;

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

                int m = (int)approxCtr.size();
                vector<cv::Mat> vertexesPl0;
                for(int i=0; i<m; i++){
                    cout<<"____ vert"<<i<<" = "<<approxCtr[i]<<endl;
//                    cout<<"dist_px("<<i<<", "<<(i+1)%m<<") = "<<norm(approxCtr[i] - approxCtr[(i+1)%m])<<endl;
                    cv::Mat pt_px = (cv::Mat_<float>(3, 1) << approxCtr[i].x, approxCtr[i].y, 1.);
                    cv::circle(imCtrs, cv::Point(approxCtr[i].x, approxCtr[i].y), 2, cv::Scalar(10, 10, 200), 2);
//                    float distPl12Cam = pAcq.getDistPlane2Cam() - (float)5.8;
//                    cout<<"distPl12Cam = "<<distPl12Cam<<endl;
                    vertexesPl0.push_back(pAcq.imProj2Plane(pt_px));
                    cout<<"\t"<<vertexesPl0[i]<<endl;
                }

                cv::imwrite("imCtrs.png", imCtrs);

                _listObj.push_back(recogObj(vertexesPl0, (eObjCol)col));
            }
        }
    }


    std::cout<<"_________Process ProcIObj: end____________"<<std::endl;
}

int ProcIDObj::loadListObj(const std::string& objPlgrdFile){
    // TODO: process color rom the file
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
            setColors(ObjCol, obj);

            _listRefObj.push_back(new Play_Obj(sandCube, parallelepiped, dim, ObjCol));
            break;
        case sandCone:
            nbPara++;
            obj = obj.substr(string("sandCone").length()+1);
            setDim(dim, obj, 2);
            setColors(ObjCol, obj);

            _listRefObj.push_back(new Play_Obj(sandCone, parallelepiped, dim, ObjCol));
            break;
        case sandCyl:
            nbCylinder++;
            obj = obj.substr(string("sandCyl").length()+1);

            setDim(dim, obj, 2);
            setColors(ObjCol, obj);

            _listRefObj.push_back(new Play_Obj(sandCyl, cylinder, dim, ObjCol));
            break;
        case shellGreen:
            nbCylinder++;
            obj = obj.substr(string("shellGreen").length()+1);

            setDim(dim, obj, 2);
            setColors(ObjCol, obj);

            _listRefObj.push_back(new Play_Obj(shellGreen, cylinder, dim, ObjCol));
            break;
        case shellViolet:
            nbCylinder++;
            obj = obj.substr(string("shellViolet").length()+1);

            setDim(dim, obj, 2);
            setColors(ObjCol, obj);

            _listRefObj.push_back(new Play_Obj(shellViolet, cylinder, dim, ObjCol));
            break;
        case shellWhite:
            nbCylinder++;
            obj = obj.substr(string("shellWhite").length()+1);

            setDim(dim, obj, 2);
            setColors(ObjCol, obj);

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

void ProcIDObj::setColors(eObjCol c, string& s){
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

    if(! isColObjExist(c))
        _listColObj.push_back(std::make_tuple(c, hsv_min, hsv_max));
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

bool ProcIDObj::isColObjExist(const eObjCol c){
    std::vector<std::tuple<eObjCol, cv::Scalar, cv::Scalar>>::iterator it = _listColObj.begin();

    for(; it != _listColObj.end(); ++it){
        if(get<0>(*it)  == c)
            return true;
    }

    return false;
}

cv::Mat ProcIDObj::getBinaryImage(cv::Mat m, int col){
    cv::Mat ret;
    cv::Scalar hsv_min = cv::Scalar(get<1>(_listColObj[col]));
    cv::Scalar hsv_max = cv::Scalar(get<2>(_listColObj[col]));
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
    approxPolyDP(cv::Mat(ctr), approxCtr, arcLength(cv::Mat(ctr), true) * 0.02, true);
}

/**
 * @description: Try to identify the shape describe by the contour "ctr".
 *      If the shape is recognized then set the type of object in "t",
 *      otherwise "t" is equal to the possible maximum number of type of object.
 * @note: For now only recognize a quadrilateral
 */
eObjShape ProcIDObj::recogShape(const vector<cv::Mat>& vertexes, vector<Vector3D<float>>& edges){
    eObjShape t = objShapeMax;
    edges.clear();
    int s = (int)vertexes.size();

    if(s == 4){  // Recognition of a quadrilateral
        float epsAngl = 0.05; // 0.05% <=> a margin of 16.5°/360°
        // Compute the sum of the three first angles
        float angles = 0.;
        for(int i=0; i<s; i++){
            edges.push_back(Vector3D<float>(cv::Mat(vertexes[i] - vertexes[(i+1)%s])));
            if(i>0){
                cout<<"edges["<<i<<"] = "<<edges[i]<<endl;
                cout<<"edges["<<(i+1)%s<<"] = "<<edges[(i+1)%s]<<endl;
                angles += edges[i].angle(edges[(i-1)%s]);
                cout<<"angles = "<<angles<<endl;
            }
        }
        cout<<"angles = "<<angles<<endl;
        if(fabsf(angles) < (2*M_PI-M_PI_2 + epsAngl)  &&  fabsf(angles) > (2*M_PI-M_PI_2 - epsAngl)){
            cout<<"Found quadrilateral\n";
            t = parallelepiped;
        }

        // TODO: Add test to distinguish the quadrilateral: square, rectangle, ...
    }
    else{
        cout<<"Shape not reconized\n";
    }

    return t;
}

Play_Obj *ProcIDObj::recogObj(vector<cv::Mat>& vertexes, eObjCol col){
    vector<Vector3D<float>> edges;

    eObjShape shape = recogShape(vertexes, edges);

    std::vector<float> dim;
    recogObj(vertexes, col, shape);

    eObjType objType;

    return new Play_Obj(objType, shape, dim, col);
}

void ProcIDObj::recogObj(vector<cv::Mat>& vertexes, eObjCol col, eObjShape shape){
    vector<Play_Obj*> listObj(getSameInListRefObj(col, shape));

    cout<<"recoObj(3p): listObj.size() = "<<listObj.size()<<endl;
    for(int i=0; i<(int)listObj.size(); i++){
        listObj[i]->print();
    }

//    return new Play_Obj(sandCube, parallelepiped, vector<float>, col);
}

vector<float> ProcIDObj::getPosOfShape(vector<cv::Mat>& ctr, eObjShape t){
    vector<float> pos;
    switch(t){
    case parallelepiped:
        Play_Obj obj();

        break;
    case cone:

        break;
    case cylinder:

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
        for(it = _listRefObj.begin(); it != _listRefObj.end(); ++it){
            if((*it)->isDimEqual(dim, epsDim))
                listObj.push_back(*it);
        }
    }

    if(shape != objShapeMax){
        std::vector<Play_Obj*> listObjTemp(listObj);
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


