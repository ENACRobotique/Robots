/*
 * Inbox.h
 *
 *  Created on: 15 mai 2015
 *      Author: seb
 */

#ifndef MAIN_AI_TOOLS_INBOX_H_
#define MAIN_AI_TOOLS_INBOX_H_

#include "message_header.h"
#include "GeometryTools.h"

class Inbox {
    public:
        Inbox();
        ~Inbox();

        void checkInbox();
        bool lastGoal(Point2D<float>& goal);

    private:
        Point2D<float>  posGoal;
        bn_Address      addrProp;
};

#endif /* MAIN_AI_TOOLS_INBOX_H_ */
