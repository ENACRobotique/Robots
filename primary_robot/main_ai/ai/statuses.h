/*
 * statuses.h
 *
 *  Created on: 21 févr. 2015
 *      Author: seb
 */

#ifndef AI_STATUSES_H_
#define AI_STATUSES_H_

#include <deque>
#include <array>

#include "messages-statuses.h"
#include "math_types.h"

using namespace std;

class Statuses {
    public:
        Statuses();
        ~Statuses();

        void maintenace();
        int receivedNewStatus(sGenericStatus &status);

        sGenericStatus& getLastStatus(eElement el, frame_t fr = FRAME_PLAYGROUND);

        //Simple function define in the FRAME_PLAYGROUND
        sPt_t getPosXY(eElement el);

    private:
        void fromPRPG2PG(s2DPosAtt *srcPAPR, s2DPAUncert *srcUPR, s2DPosAtt *srcPAPG, s2DPAUncert *srcUPG, s2DPosAtt *dstPAPG, s2DPAUncert *dstUPG);

        array <deque <sGenericStatus>, NUM_E_ELEMENT> _list;

};

#endif /* AI_STATUSES_H_ */
