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
#include "GeometryTools.h"

using namespace std;

typedef struct{
    bool updated; //updated the position in monitoring hmi only for the robot
    //TODO time maintenance

}statusConfig_t;

class Statuses {
    public:
        Statuses();
        ~Statuses();
        void maintenace();

        void setConfig();


        int receivedNewStatus(sGenericStatus &status);

        sGenericStatus& getLastStatus(eElement el, frame_t fr = FRAME_PLAYGROUND);

        //Simple function define in the FRAME_PLAYGROUND
        Point2D<float> getLastPosXY(eElement el);
        float getLastOrient(eElement el);
        float getLastSpeed(eElement el);

    private:
        void fromPRPG2PG(s2DPosAtt *srcPAPR, s2DPAUncert *srcUPR, s2DPosAtt *srcPAPG, s2DPAUncert *srcUPG, s2DPosAtt *dstPAPG, s2DPAUncert *dstUPG);

        array <deque <sGenericStatus>, NUM_E_ELEMENT> _list;
        array <statusConfig_t, NUM_E_ELEMENT> _config;

};

#endif /* AI_STATUSES_H_ */
