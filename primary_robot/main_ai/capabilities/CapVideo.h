/*
 * CapVideo.h
 *
 *  Created on: 12 avr. 2015
 *      Author: Sébastien Malissard
 */

#ifndef CAPABILITIES_CAPVIDEO_H_
#define CAPABILITIES_CAPVIDEO_H_

#include <Capability.h>

class CapVideo : public Capability {
    public:
        CapVideo(Robot* robot_init) : Capability(robot_init){}
        ~CapVideo(){}


};

#endif /* CAPABILITIES_CAPVIDEO_H_ */
