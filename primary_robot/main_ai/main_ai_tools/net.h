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

class Net {
    public:
        Net();
        ~Net();
        void maintenace();
        void clearEl();

        void sendPath(deque <sTrajEl_t> &trajEl);
        void sendPathOrient(deque <sTrajOrientEl_t> &trajElOrient);


    private:
        void convTrajToTrajOrient();
        void sendPathToNet();
        void sendPathOrientToNet();

        static unsigned int _tid; // common to all Net instances
        unsigned int _sid;
        queue <sTrajEl_t> _trajEl;
        queue <sTrajOrientEl_t> _trajOrientEl;

};

#endif /* MAIN_AI_TOOLS_NET_H_ */
