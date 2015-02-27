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

        void sendPath(vector <sTrajEl_t> &trajEl);
        //void sendPathOrient(vector <sTrajElOrient_t> &trajEl);


    private:
        void sendPath2Net();
        queue <sTrajEl_t> _trajEl;

};

#endif /* MAIN_AI_TOOLS_NET_H_ */
