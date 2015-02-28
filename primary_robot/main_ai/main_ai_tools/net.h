/*
 * net.h
 *
 *  Created on: 24 févr. 2015
 *      Author: seb
 */

#ifndef MAIN_AI_TOOLS_NET_H_
#define MAIN_AI_TOOLS_NET_H_

#include <stack>
#include <queue>

#include "messages-locomotion.h"
#include "path.h"

using namespace std;

typedef struct{
    float t1;   // in us (synchronized time):
    float t2;

    sPt_t p1;
    sPt_t p2;

    sObs_t obs;//TODO change in circle type with the new math tools

    float theta1;
    float theta2;

    float arc_len;
    float seg_len;

    bool rot1_dir; // (false: CW | true: CCW)
    bool rot2_dir;
}sTrajOrientEl_t;

class Net {
    public:
        Net();
        ~Net();
        void maintenace();
        void clearEl();

        void sendPath(vector <sTrajEl_t> &trajEl);
        void sendPathOrient(vector <sTrajOrientEl_t> &trajElOrient);


    private:
        void convTrajToTrajOrient();
        void sendPathToNet();
        void sendPathOrientToNet();

        queue <sTrajEl_t> _trajEl;
        queue <sTrajOrientEl_t> _trajOrientEl;

};

#endif /* MAIN_AI_TOOLS_NET_H_ */
