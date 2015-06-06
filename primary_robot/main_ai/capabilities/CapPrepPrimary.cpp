/*
 * CapPrepPrimary.cpp
 *
 *  Created on: 18 avr. 2015
 *      Author: Sébastien Malissard
 */

#include "CapPrepPrimary.h"
#include "CapPosition.h"
#include "CapIO.h"
#include "CapTeam.h"
#include "CapAI.h"
#include "tools.h"
#include "communications.h"
extern "C"{
#include "millis.h"
}

#define MINVARIANCE_XY (5e-4) // (cm²)
#define MAXVARIANCE_XY (2e3) // (cm²)
#define MINVARIANCE_THETA (2e-3) // (rad²)
#define MAXVARIANCE_THETA (500) // (rad²)

int CapPrepPrimary::loop(){
    CapIO* capIO = dynamic_cast<CapIO*> (robot->caps[eCap::IO]);
    CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);
    CapPosition* capPos = dynamic_cast<CapPosition*> (robot->caps[eCap::POS]);
    static std::vector<SimpleTraj> traj;
    static int index = 0;
     static bool wait = false;
     static unsigned int timeSave=0;
    if(!capIO){
        logs << ERR << "Primary must be have an HMI interface real or simulate";
        exit(EXIT_FAILURE);
    }

    switch (_step){
    case Step::COLOR_SELECTION: //Choose the color and take off the starting cord
        capIO->selectColor();

        if(capIO->getHMI(IHM_STARTING_CORD) == CORD_IN) {
          //  getchar();
            //logs << INFO << "getchar";
            float theta_robot;
            Point2D<float> pos_robot;

            if (capTeam->getColor() == eColor_t::YELLOW) {
                logs << INFO << "Color selected is YELLOW";
                pos_robot = {INIT_POS_YELLOW_X, INIT_POS_YELLOW_Y};
                theta_robot = INIT_ANGLE_YELLOW;
            }
            else if (capTeam->getColor() == eColor_t::GREEN) {
                logs << INFO << "Color selected is GREEN";
                pos_robot = {INIT_POS_GREEN_X, INIT_POS_GREEN_Y};
                theta_robot = INIT_ANGLE_GREEN;
            }
            else {
                logs << ERR << "Error selection color";
                return -1;
            }
            theta_robot = 0;
            sendSetPosPrimary(pos_robot, theta_robot, 10.*10., 10.*10., 0., M_PI*M_PI/(10.*10.));            // Sending approximate initial position to the propulsion

            //TODO procedure de mise en place
            std::vector<Segment2D<float>> robotSeg;
            Segment2D<float> seg = {{10.,10.},{10.,-10.}};
            robotSeg.push_back(seg);
            std::vector<Segment2D<float>> playgroundSeg;
            Segment2D<float> pg1, pg2;
            if(capTeam->getColor() == eColor_t::GREEN){
                pg1= {{270.,80.},{280.,80.}};
                pg2 = {{295.,80.},{295.,120.}};
            }
            else{
                pg1 = {{30.,80.},{20.,80.}};
                pg2 = {{5.,80.},{5.,120.}};
            }
            playgroundSeg.push_back(pg1);
            playgroundSeg.push_back(pg2);

            setStartingPosition(traj, pos_robot, theta_robot, pos_robot /*destPt*/, theta_robot /*destAngle*/, robotSeg, playgroundSeg);

            logs << INFO << "\n";
            for(SimpleTraj i : traj){
                logs << fixed << setprecision(2) << "Pt:" << i.dest << " angle:" << i.angle*180/M_PI << " action:" << i.action << "\n";
            }

            logs << INFO << "End step color selection";
            _step = Step::WAIT_POS_PROP;
        }
        break;

    case Step::WAIT_POS_PROP:
        if(capPos->getLastPosXY().x != 0 && capPos->getLastPosXY().y != 0){
            robot->env->obs[0].c.x = capPos->getLastPosXY().x;
            robot->env->obs[0].c.y = capPos->getLastPosXY().y;
            _step = Step::WAIT_STARTING_CORD;
        }
        break;

    case Step::WAIT_STARTING_CORD: //Wait to take in the starting cord
        if(capIO->getHMI(IHM_STARTING_CORD) == CORD_IN){
            if(CapAI* capAI = dynamic_cast<CapAI*> (robot->caps[eCap::AI]))
                capAI->initObjective();

            logs << INFO << "End step wait starting cord";
            _step = Step::WAIT_INIT_POS;
        }
        break;

    case Step::WAIT_INIT_POS:
        if(wait){
            if(millis() - timeSave > 2000){
                wait = false;
             //   logs << ERR << "getchar";
  //  getchar();

                if(traj[index].action == 1){
                    Point2D<float> pos = capPos->getLastPosXY();
                    pos.x = traj[index].dest.x;
                    sendMixPosPrimary(pos, traj[index].angle, MINVARIANCE_XY, MAXVARIANCE_XY, 0., MINVARIANCE_THETA);
                    //recale x
                }
                else if(traj[index].action == 2){
                    Point2D<float> pos = capPos->getLastPosXY();
                    pos.y = traj[index].dest.y;
                    sendMixPosPrimary(pos, traj[index].angle, MINVARIANCE_XY, MAXVARIANCE_XY, -M_PI/2, MINVARIANCE_THETA);
                    //recale y
                }
                //Wait new position was send

                path.go2PointOrient(traj[index].dest, robot->env->obs, traj[index].angle);
            }
        }
        else{
            if(capPos->getLastPosXY().distanceTo(traj[index].dest) < 0.5){
                robot->env->obs[0].c.x = capPos->getLastPosXY().x;
                robot->env->obs[0].c.y = capPos->getLastPosXY().y;
                if(traj[index].action == 3){
                    _step = Step::WAIT_START;
                    break;
                }
             /*   else if(traj[index].action == 1){
                    Point2D<float> pos = capPos->getLastPosXY();
                    pos.x = traj[index].dest.x;
                    sendMixPosPrimary(pos, traj[index].angle, MINVARIANCE_XY, MAXVARIANCE_XY, 0., MINVARIANCE_THETA);
                    //recale x
                }
                else if(traj[index].action == 2){
                    Point2D<float> pos = capPos->getLastPosXY();
                    pos.y = traj[index].dest.y;
                    sendMixPosPrimary(pos, traj[index].angle, MINVARIANCE_XY, MAXVARIANCE_XY, -M_PI/2, MINVARIANCE_THETA);
                    //recale y
                }*/
                else  if(traj[index].action == 5){
                    wait = true;
                    timeSave = millis();

                }

                //Wait new position was send
                index++;
                path.go2PointOrient(traj[index].dest, robot->env->obs, traj[index].angle);
            }
        }
        break;

    case Step::WAIT_START: //Wait the start (take off the starting cord)
        //logs << INFO << "getchar";
        //getchar();
        if (capIO->getHMI(IHM_STARTING_CORD) == CORD_OUT) {
            capTeam->setStartGame(millis());

            logs << INFO << "End step wait start";
            _step = Step::GAME;
        }
        break;

    case Step::GAME: //Let's go
        logs << INFO << "Start game : Let's go!!!!";
        return 1;
    }

    return 0;
}
