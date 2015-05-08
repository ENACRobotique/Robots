/*
 * CapPosSimuSecondary.cpp
 *
 *  Created on: 13 avr. 2015
 *      Author: seb
 */


#include "CapPosSimuSecondary.h"
#include "CapTeam.h"

extern "C"{
#include "millis.h"
}

Point2D<float> CapPosSimuSecondary::getLastPosXY(){
    static vector<Point2D<float>> trjS{{ 10., 100. }, { 100., 100. }, { 125., 120. }, { 125., 190. }}; //trajectory of the secondary robot : Yellow
    static bool update = true;
    static bool first = true;
    static unsigned int state = 0;
    static unsigned int lastTime = 0;
    static unsigned int lastTime2 = 0;
    static unsigned int startTime = 0;
    static unsigned int waitTime = 0;
    static unsigned int diffTime = 0;
    static bool wait = false;
    static Point2D<float> pos;
    static Vector2D<float> spd;
    CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);

    if (state < trjS.size() - 1) {
        if(first) {
            if(capTeam->getColor() == GREEN) {
                for(Point2D<float>& pt : trjS)
                    pt.x = 300. - pt.x;
            }

            startTime = millis();
            lastTime = startTime + START_DELAY;
            first = false;
        }

        unsigned int time = millis() - diffTime;

        if(colissionDetection(robot->el, robot->env->obs)){
            if(!wait){
                wait = true;
                waitTime = millis();
            }
            lastTime2 = millis();
        }
        else if(wait){
            if(millis() - lastTime2 > RESTART_DELAY){
                wait = false;
                diffTime += millis() - waitTime;
            }
        }
        else if(time - startTime > START_DELAY){
            if(update) {
                pos = trjS[state];
                float theta, ctheta, stheta;
                theta = atan2f((trjS[state + 1].y - trjS[state].y), (trjS[state + 1].x - trjS[state].x));
                sincosf(theta, &stheta, &ctheta);
                spd = Vector2D<float>(ctheta, stheta) * SPEED_SECONDARY;

                update = false;
            }

            pos += spd * (int)(time - lastTime) / 1000.;

            if (pos.distanceSqTo(trjS[state + 1]) < 1) {
                state++;
                update = true;
            }

            lastTime = millis() - diffTime;
        }
    }

    return pos;
}

