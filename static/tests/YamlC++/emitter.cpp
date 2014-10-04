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


#ifdef PATH
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "types.h"
#endif


using namespace YAML;
using namespace std;


#ifdef EXEMPLE

struct Position {
    float x; // (cm)
    float y; // (cm)
    float theta; // (°)
};

Emitter& operator <<(Emitter& out, const Position& v) {
    out << BeginSeq << v.x << Comment("(cm)") << v.y << Comment("(cm)")
            << v.theta << Comment("(°)") << EndSeq;
    return out;
}

int main() {
    Position v { 1, 2, 3 };

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

Emitter& operator << (Emitter& out, const sPt_t& pt){
    out << Flow << BeginSeq;
    out << pt.x << pt.y;
    out << EndSeq;
    return out;
}

Emitter& operator << (Emitter& out, const sObs_t& obs){
    out << Flow << BeginSeq;
    out << obs.c << obs.r << obs.moved << obs.active << obs.state;
    out << EndSeq;
    return out;
}

Emitter& operator << (Emitter& out, const sTrajEl_t& ele){
    out << Flow << BeginSeq;
    out << ele.p1 << ele.p2 << ele.obs << ele.arc_len << ele.seg_len << ele.sid;
    out << EndSeq;
    return out;
}

Emitter& operator << (Emitter& out, const sPathHeader_t& hea){
    out << Flow << BeginSeq;
    out << BeginMap << Key << "dist" << Value << hea.dist << EndMap;
    out << BeginMap << Key << "tid" << Value << hea.tid << EndMap;
    out << BeginMap << Key << "path_len" << Value << hea.path_len << EndMap;
    out << EndSeq;
    return out;
}

Emitter& operator << (Emitter& out, const sPathDispaly_t& dis){
    out  << BeginSeq;
        out << BeginMap << Key << "Header" << Value << dis.header << EndMap;
        out << BeginMap << Key << "Element" << Value << BeginSeq;
            for(unsigned int i = 0 ; i < dis.header.path_len ; i++){
                out << BeginMap << Key << i << Value << dis.path[i] << EndMap;
            }
        out << EndSeq;
    out << EndSeq;

    return out;
}



sTrajEl_t tabEl[4]={ //Trajectory for tree upwards vertical in the origin of the table for tree
    {{0.  ,  0.},{15.2,-16.9},{{33.2 ,-16.9}, 18. , 0, 1}, 0. , 0., 0},
    {{17.3,-8.5},{17.3, -8.5},{{-4.9 ,  3.4},-25.2, 0, 1}, 0. , 0., 1},
    {{17.3,15.2},{17.3, 15.2},{{33.3 , 23.7}, 18.1, 0, 1}, 0. , 0., 2},
    {{15.2,23.7},{15.2, 23.7},{{15.2 , 23.7}, 0.  , 0, 1}, 0. , 0., 3}
    };

int main(){
    sPath_t path;
    sTrajEl_t *tab = NULL;

    /*Create path*/
    path.dist = 0;
    path.path_len = 4;
    path.tid = 0 ;
    unsigned int i;

    if ((tab = (sTrajEl_t *) malloc(sizeof(tabEl[0])*path.path_len)) == NULL)
        printf("Error : malloc()\n");
    memcpy(tab,&tabEl[0], sizeof(tabEl[0])*path.path_len);

    path.path = tab;

    sPathHeader_t header = {path.dist, path.tid , path.path_len};

    sPathDispaly_t display;
    display.header = header;
    display.path = tab;

    /*Save data*/
    Emitter out;
    out.SetFloatPrecision(3);
    out.SetDoublePrecision(3);



    out << BeginSeq << BeginMap << Key << "Trajectory" << Value;
    {
        out << BeginSeq << BeginMap << Key << "1" << Value;
        {
            out  << BeginSeq << BeginMap << Key << "Header" << Value ;
            {
                out << header;
            }
            out << EndMap;
            out << BeginMap << Key << "Element" << Comment("[p1:[x, y] , p2:[x, y] , obs:[c:[x, y], r , moved, active, state], arc_len, seg_len, sid]") << Value;
            {
                out << BeginSeq;
                for(i = 0 ; i < path.path_len ; i++){
                    out << BeginMap << Key << i << Value << tab[i] << EndMap;
                }
                out << EndSeq;
            }
            out << EndMap;
            out << EndSeq;
        }
        out << EndMap ;

        /*****/

        out << BeginMap << Key << "2" << Value << BeginSeq;
                {
                    out  << BeginMap << Key << "Tid" << Value << Flow << BeginSeq;
                    {
                        out << BeginMap << Key << "tid" << Value << 2 << EndMap;
                        out << BeginMap << Key << "len" << Value << 4 << EndMap;
                    }
                    out << EndSeq << EndMap;
                    out << BeginMap << Key << "Sid" << Value << BeginSeq;
                    {
                        out << BeginMap << Key << "1" << Value << Flow << BeginSeq ;
                        {
                            out << BeginMap << Key << "sid" << Value << 1 << EndMap;
                            out << BeginMap << Key << "p1" << Value << 10.2 << EndMap;
                        }
                        out << EndSeq <<EndMap;
                        out << BeginMap << Key << "2" << Value << Flow << BeginSeq ;
                        {
                            out << BeginMap << Key << "sid" << Value << 1 << EndMap;
                            out << BeginMap << Key << "p1" << Value << 10.2 << EndMap;
                        }
                        out << EndSeq <<EndMap;
                    }
                    out << EndSeq <<EndMap;
                }
                out << EndSeq << EndMap;

        /*****/

        out << BeginMap << Key << "3" << Value;
            out << display;
        out << EndMap;
    }

    out << EndMap;
    //Other information : speed, init...
    out << EndSeq;

    cout << "Here's the output YAML:\n" << out.c_str() << endl;

    return 0;
}

#endif


