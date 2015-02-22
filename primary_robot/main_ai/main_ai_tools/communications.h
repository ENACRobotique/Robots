/*
 * communications_tools.h
 *
 *  Created on: 22 févr. 2015
 *      Author: seb
 */

#ifndef COMMUNICATIONS_TOOLS_H_
#define COMMUNICATIONS_TOOLS_H_

#include <fstream>

#include "math_types.h"

using namespace std;

//Send message
void sendObsCfg();
void sendObss();

void ping();

//Receive message
void checkInbox(int verbose, ofstream &file);
bool lastGoal(sPt_t &goal, bool get);


#endif /* COMMUNICATIONS_TOOLS_H_ */
