/*
 * emitter.cpp
 *
 *  Created on: 2 oct. 2014
 *      Author: seb
 */

//#define EXEMPLE
#define PATH

#include <yaml-cpp/emitter.h>
#include <iostream>
#include <iomanip>
#include <sstream>

#ifdef PATH
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "types.hpp"
#endif

using namespace YAML;
using namespace std;

#ifdef EXEMPLE

struct Position {
    float x; // (cm)
    float y;// (cm)
    float theta;// (°)
};

Emitter& operator <<(Emitter& out, const Position& v) {
    out << BeginSeq << v.x << Comment("(cm)") << v.y << Comment("(cm)")
    << v.theta << Comment("(°)") << EndSeq;
    return out;
}

int main() {
    Position v {1, 2, 3};

    Emitter out;
    out << BeginSeq;
    {
        out << "Hello, World!" << Comment("prints Hello, World!");
        {
            out << BeginMap << Key << "blah" << Value << v << EndMap;
        }
    }
    out << EndSeq;

    cout << "Here's the output YAML:\n" << out.c_str(); // prints "Hello, World!"
    return 0;
}
#endif

#ifdef PATH

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

Emitter& operator <<(Emitter& out, const sTrajEl_t& ele) {
    out << Flow << BeginMap;
    out << Key << "start" << Value << ele.p1;
    out << Key << "end" << Value << ele.p2;
    out << Key << "obs" << Value << ele.obs;
    // << Comment(ele.arc_len); //"ele.arc_len << ele.seg_len << ele.sid");
    out << EndMap;

    return out;
}

Emitter& operator <<(Emitter& out, const sPath_t& p) {
    out << BeginMap;
    {
        out << Key << "Header" << Value;
        {
            out << Flow << BeginMap;
            {
                out << Key << "tid" << Value << p.tid;
            }
            out << EndMap;

            {
                ostringstream ss;
                ss << "dist: " << p.dist << " cm, path_len: " << p.path_len;
                out << Comment(ss.str());
            }
        }

        out << Key << (p.path_len > 1 ? "Elements" : "Element")/* << Comment("[p1:[x, y] , p2:[x, y] , obs:[c:[x, y], r , moved, active, state], arc_len, seg_len, sid]")*/<< Value;
        {
            out << BeginSeq;
            for (unsigned int i = 0; i < p.path_len; i++) {
                out << p.path[i];
            }
            out << EndSeq;
        }
    }
    out << EndMap;

    return out;
}

sTrajEl_t tabEl[4] = { //Trajectory for tree upwards vertical in the origin of the table for tree
{ { 0., 0. }, { 15.2, -16.9 }, { { 33.2, -16.9 }, 18., 0, 1 }, 0., 0., 0 }, //
{ { 17.3, -8.5 }, { 17.3, -8.5 }, { { -4.9, 3.4 }, -25.2, 0, 1 }, 0., 0., 1 }, //
{ { 17.3, 15.2 }, { 17.3, 15.2 }, { { 33.3, 23.7 }, 18.1, 0, 1 }, 0., 0., 2 }, //
{ { 15.2, 23.7 }, { 15.2, 23.7 }, { { 15.2, 23.7 }, 0., 0, 1 }, 0., 0., 3 } };

int main() {
    sPath_t path;

    /*Create path*/
    path.dist = 42;
    path.path = tabEl;
    path.path_len = 4;
    path.tid = 0;

    /*Save data*/
    Emitter out;
    out.SetFloatPrecision(3);
    out.SetDoublePrecision(3);

    out << BeginMap;
    {
        out << Key << "Header" << Value;
        out << BeginMap;
        {
            out << Key << "version" << Value << "0.1";
        }
        out << EndMap << Newline;

        out << Key << "Trajectory" << Value;
        out << BeginSeq;
        {
            out << path;

            path.tid++;
            path.path_len = 1;
            out << path;
        }
        out << EndSeq;

        //Other information : speed, init...
    }
    out << EndMap;

    cout << "Here's the output YAML:\n" << out.c_str() << endl;

    return 0;
}

#endif

