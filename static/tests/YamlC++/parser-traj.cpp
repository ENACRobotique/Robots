/*
 * parser-traj.cpp
 *
 *  Created on: 8 oct. 2014
 *      Author: seb
 */


#include <yaml-cpp/emittermanip.h>
#include <yaml-cpp/node.h>
#include <yaml-cpp/nodeimpl.h>
#include <yaml-cpp/parser.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <malloc.h>
#include <math.h>

#include "types.hpp"

using namespace YAML;

//// our data types
//struct Vec3 {
//    float x, y, z;
//};
//
//struct Power {
//    std::string name;
//    int damage;
//};
//
//struct Monster {
//    std::string name;
//    Vec3 position;
//    std::vector<Power> powers;
//};
//
//// now the extraction operators for these types
//void operator >>(const Node& node, Vec3& v) {
//    node[0] >> v.x;
//    node[1] >> v.y;
//    node[2] >> v.z;
//}
//
//void operator >>(const Node& node, Power& power) {
//    node["name"] >> power.name;
//    node["damage"] >> power.damage;
//}
//
//void operator >>(const Node& node, sPath_t& path) {
//    const YAML::Node& traj = node["Trajectory"];
//    for (unsigned int i = 0; i < 2; i++){ //FIXME
//        node
//    }
//   /* node["position"] >> monster.position;
//    const Node& powers = node["powers"];
//   for (unsigned i = 0; i < powers.size(); i++) {
//        Power power;
//        powers[i] >> power;
//        monster.powers.push_back(power);
//    }*/
//}








void operator >>(const Node& out, sPt_t& pt) {
    out["x"] >> pt.x;
    out["y"] >> pt.y;
}

void operator >>(const Node& out, sObs_t& obs) {
    out["c"] >> obs.c; // << (bool) obs.moved << (bool) obs.active << (bool) obs.state;
    out["r"] >> obs.r;
}

void operator >>(const Node& out, sTrajEl_t& ele) {
    out["start"] >> ele.p1;
    out["end"] >> ele.p2;
    out["obs"] >> ele.obs;
    ele.arc_len = -1;
    ele.seg_len = ele.p1.getDistTo(ele.p2);
}

void operator >>(const Node& out, sPath_t& p) {
    const Node& header = out["Header"];
    header["tid"] >> p.tid;
    p.dist = -1;

    const Node& elements = out["Elements"];// == NULL ?: out["Element"];

    p.path_len = elements.size();
    p.path = (sTrajEl_t*)malloc(p.path_len*sizeof(*p.path));

    for(unsigned int i = 0; i < p.path_len; i++){
        elements[i] >> p.path[i];
        p.path[i].sid = i;
    }
}

int main() {
    std::ifstream fin("log3.yaml");
    Parser parser(fin);
    Node doc;
    parser.GetNextDocument(doc);

    const Node& trajectories = doc["Trajectory"];
    for (unsigned i = 0; i < trajectories.size(); i++) {
        sPath_t path;
        trajectories[i] >> path;
        std::cout << "tid" << path.tid << std::endl;
        std::cout << "x" << path.path[0].p1.x << std::endl;
    }

    return 0;
}
