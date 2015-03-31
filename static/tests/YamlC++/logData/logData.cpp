/*
 * log.cpp
 *
 *  Created on: 10 nov. 2014
 *      Author: Sébastien Malissard
 */

#include "logData.hpp"

#include <fstream>
#include <sstream>


using namespace YAML;
using namespace std;

/****Emitter operator****/

Emitter& operator <<(Emitter& out, const sPt_t& pt) {
    out << Flow << BeginMap;
    out << Key << "x" << Value << pt.x;
    out << Key << "y" << Value << pt.y;
    out << EndMap;

    return out;
}

Emitter& operator <<(Emitter& out, const sObs_t& obs) {
    out << Flow << BeginMap;
    out << Key << "c" << Value << obs.c; // << (bool) obs.moved << (bool) obs.active << (bool) obs.state;
    out << Key << "r" << Value << obs.r;
    out << EndMap;

    return out;
}

Emitter& operator <<(Emitter& out, const sTrajElLog& ele) {
    out << Flow << BeginMap;
    out << Key << "date" << Value << ele.date;
    out << Key << "start" << Value << ele.el.p1;
    out << Key << "end" << Value << ele.el.p2;
    out << Key << "obs" << Value << ele.el.obs;
    out << EndMap;

    ostringstream ss; //comment
    ss << "sid: " << ele.el.sid << ", ";
    ss << "arc_len: " << ele.el.arc_len << ", ";
    ss << "seg_len: " << ele.el.seg_len << ", ";
    ss << "nbTry: " << ele.nbSend;
    out << Comment(ss.str());

    return out;
}

Emitter& operator <<(Emitter& out, const sPathLog& p) {
    out << BeginMap;
    {
        out << Key << "Header" << Value;
        {
            out << Flow << BeginMap;
            {
                out << Key << "tid" << Value << p.path.tid;
            }
            out << EndMap;

            {
                ostringstream ss;
                ss << "dist: " << p.path.dist << " cm, path_len: " << p.path.path_len;
                out << Comment(ss.str());
            }
        }

        out << Key << "Elements" /* << Comment("[p1:[x, y] , p2:[x, y] , obs:[c:[x, y], r , moved, active, state], arc_len, seg_len, sid]")*/<< Value;
        {
            out << BeginSeq;
            for (unsigned int i = 0; i < p.listEl.size(); i++) {
                out << p.listEl[i];
            }
            out << EndSeq;
        }
    }
    out << EndMap;

    return out;
}


/****Parser operator****/

void operator >>(const Node& out, sPt_t& pt) {
    out["x"] >> pt.x;
    out["y"] >> pt.y;
}

void operator >>(const Node& out, sObs_t& obs) {
    out["c"] >> obs.c; // << (bool) obs.moved << (bool) obs.active << (bool) obs.state;
    out["r"] >> obs.r;
}

void operator >>(const Node& out, sTrajElLog& ele) {
    out["date"] >> ele.date;
    out["start"] >> ele.el.p1;
    out["end"] >> ele.el.p2;
    out["obs"] >> ele.el.obs;
}


/****Constructor****/

logData::logData(string file):fileLog(file) {
    header.version = 1.0;
}


/****File management****/

void logData::newFile(std::string file){
    fileLog = file;
}

void logData::clean(){ //TODO Can probably be improved
    while(!listPath.empty()){
        while(!listPath.front().listEl.empty()){
            listPath.front().listEl.pop_front();
        }
        listPath.pop_front();
    }
}

int logData::save() {
    ofstream file(fileLog.c_str());
    Emitter out;

    out.SetFloatPrecision(3);
    out.SetDoublePrecision(3);

    out << BeginMap;
    headerEmitter(header, out);
    out << Newline;
    pathEmitter(listPath, out);
    out << EndMap;

    file << out.c_str();

    return 0;
}

int logData::load() {
    ifstream file(fileLog.c_str());
    Parser in(file);
    Node doc;

    clean();
    in.GetNextDocument(doc);
    headerParser(doc);
    pathParser(doc);
    return 0;
}


/****Addition****/

int logData::addPath(sPath_t newPath) {
    sPathLog temp;
    temp.path = newPath;
    listPath.push_back(temp);
    return listPath.size() - 1; //return the path number
}

int logData::addEl(unsigned int num, sTrajEl_t el, float date, unsigned int nbSend) { //num is the path number
    sTrajElLog temp;
    temp.el = el;
    temp.nbSend = nbSend;
    temp.date = date;
    if (num < listPath.size()) {
        listPath[num].listEl.push_back(temp);
    }
    else {
        return -1; //Error path number path doesn't exits
    }
    return 0;
}

void logData::addColor(bool color) {
    header.color = color;
}


/****Get****/

bool logData::getColor(){
    return header.color;
}

int logData::getPath(float time, sPath_t& path, sTrajEl_t& trajEl){

    if(!listPath.empty()){
        if(listPath.front().listEl.front().date <= time){
            path = listPath.front().path;
            trajEl = listPath.front().listEl.front().el;
            listPath.front().listEl.pop_front();
            if(listPath.front().listEl.empty()){
                listPath.pop_front();
            }
            return 1;
        }
    }

    return 0;
}


/****Emitter****/

void logData::headerEmitter(sHeaderLog header, Emitter& out) {
    out << Key << "Header" << Value;
    out << BeginMap;
    out << Key << "version" << Value << header.version;
    out << Key << "color" << Value;
    if (header.color == false) { //FIXME with the good value
        out << "red";
    }
    else {
        out << "yellow";
    }
    out << EndMap;
}

void logData::pathEmitter(deque<sPathLog>& path, Emitter& out) {
    out << Key << "Trajectory" << Value;
    out << BeginSeq;
    for (unsigned int i(0); i < path.size(); i++) {
        out << path[i];
    }
    out << EndSeq;
}


/****Parser****/

void logData::headerParser(Node& doc){
    const Node& newHeader = doc["Header"];

    float ver;
    string col;

    newHeader["color"] >> col;
    newHeader["version"] >> ver;

    if(col == "red"){
        addColor(false); //FIXME put real value
    }
    else{
        addColor(true);
    }

    if(ver != header.version){
        cerr << "Warning : It is not the same version" << ver << header.version << endl;
    }
    //TODO date
}

/*void logData::pathParser(Node& doc){ //In this version the sid and tid are not in comment
    const Node& newTrajectory = doc["Trajectory"];


    for (unsigned i = 0; i < newTrajectory.size(); i++) {
        sPathLog path;
        const Node& header = newTrajectory[i]["Header"];
        header["tid"] >> path.path.tid;
        path.path.dist = -1;


        const Node& elements = newTrajectory[i]["Elements"];// == NULL ?: out["Element"];
        path.path.path_len = elements.size();

        listPath.push_back(path);

        for(unsigned int j = 0; j < path.path.path_len; j++){
            sTrajElLog el;
            elements[j] >> el;
            listPath[i].listEl.push_back(el);
        }

    }
}*/

void logData::pathParser(Node& doc){
    const Node& newTrajectory = doc["Trajectory"];

    for (unsigned i = 0; i < newTrajectory.size(); i++) {
        sPathLog path;
        const Node& header = newTrajectory[i]["Header"];
        path.path.tid = i;
        path.path.dist = -1; //FIXME Need to call a external function
        path.path.path = NULL;

        const Node& elements = newTrajectory[i]["Elements"];
        path.path.path_len = elements.size();
        listPath.push_back(path);

        for(unsigned int j = 0; j < path.path.path_len; j++){
            sTrajElLog el;
            elements[j] >> el;
            el.el.sid = j;
            el.el.arc_len = -1;
            el.el.seg_len = -1;
            el.nbSend = -1;
            listPath[i].listEl.push_back(el);
        }
    }
}
