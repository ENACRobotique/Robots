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


typedef enum{
        TRAJ1,
        WAIT_TRAJ1,
        TRAJ2,
        WAIT_TRAJ2,
        WAIT_STOP1,
        WAIT_POS,
        WAIT_TRAJ3,
        TRAJ4,
        TRAJ5,
        WAIT_TRAJ5,
        WAIT_STOP2,
        WAIT_POS2,
        POS_INIT,
        WAIT_POS_INIT
}stepInitPos;

int CapPrepPrimary::loop(){
    CapIO* capIO = dynamic_cast<CapIO*> (robot->caps[eCap::IO]);
    CapTeam* capTeam = dynamic_cast<CapTeam*> (robot->caps[eCap::TEAM]);
    CapPosition* capPos = dynamic_cast<CapPosition*> (robot->caps[eCap::POS]);
    static std::vector<SimpleTraj> traj;
  //  static int index = 0;
    // static bool wait = false;
    // static bool pos = false;
    // static unsigned int timeSave=0;
   //  static stepInitPos stepPos=TRAJ1;
    //static int ind=10;

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
         //   float theta_robot;
            Point2D<float> pos_robot;

            if (capTeam->getColor() == eColor_t::YELLOW) {
                logs << INFO << "Color selected is YELLOW";
                pos_robot = {INIT_POS_YELLOW_X, INIT_POS_YELLOW_Y};
         //       theta_robot = INIT_ANGLE_YELLOW;
            }
            else if (capTeam->getColor() == eColor_t::GREEN) {
                logs << INFO << "Color selected is GREEN";
                pos_robot = {INIT_POS_GREEN_X, INIT_POS_GREEN_Y};
             //   theta_robot = INIT_ANGLE_GREEN;
            }
            else {
                logs << ERR << "Error selection color";
                return -1;
            }
           // theta_robot = 0;
            //sendSetPosPrimary(pos_robot, theta_robot, 10.*10., 10.*10., 0., M_PI*M_PI/(10.*10.));            // Sending approximate initial position to the propulsion

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

         //   setStartingPosition(traj, pos_robot, theta_robot, pos_robot /*destPt*/, theta_robot /*destAngle*/, robotSeg, playgroundSeg);

            logs << INFO << "\n";
            for(SimpleTraj i : traj){
                logs << fixed << setprecision(2) << "Pt:" << i.dest << " angle:" << i.angle*180/M_PI << " action:" << i.action << "\n";
            }

            logs << INFO << "End step color selection";
            _step = Step::WAIT_STARTING_CORD;
           // _step = Step::WAIT_POS_PROP;
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
           // _step = Step::WAIT_START;

        }
        break;

    case Step::WAIT_INIT_POS:
    {
       // Point2D<float> pos_robot(300.-11.2, 80.-2.2-17.5);
       // float theta_robot = 48.37*M_PI/180.;

    //    Point2D<float> pos_robot(300.-7.-17.5, 80.+11.2);
     //   float theta_robot = -(90-48.37)*M_PI/180.;

      //  Point2D<float> pos_robot(INIT_POS_GREEN_X, INIT_POS_GREEN_Y);
     //   float theta_robot = (180.)*M_PI/180.;

        Point2D<float> pos_robot(300.-7.-11.2, 80.+2.7+17.5);
        float theta_robot = (-191.6)*M_PI/180.;

        sendSetPosPrimary(pos_robot, theta_robot, MINVARIANCE_XY, MINVARIANCE_XY, 0., MINVARIANCE_THETA);

        path.stopRobot(true);

        _step = Step::WAIT_START;
    }
/*

        switch(ind){
            case 10:
                sendSetPosPrimary(pos_robot, theta_robot, MINVARIANCE_XY, MINVARIANCE_XY, 0., MINVARIANCE_THETA);

                path.stopRobot(true);
                ind=0;
                break;
            case 0:
                if(capPos->getLastPosXY().x && capPos->getLastPosXY().y){
                    path.go2PointOrient({INIT_POS_YELLOW_X, INIT_POS_YELLOW_Y}, robot->env->obs, 0.);
                    ind = 1;
                }
                break;
            case 1:
                if(capPos->getLastPosXY().distanceTo({INIT_POS_YELLOW_X, INIT_POS_YELLOW_Y}) < 1.){
                    _step = Step::WAIT_START;
                }
                break;
        }





*/










        /*
        switch(stepPos){
            case TRAJ1:
                logs << INFO << "TRAJ1";
                path.go2PointOrient(traj[1].dest, robot->env->obs, traj[1].angle);
                stepPos = WAIT_TRAJ1;
                break;

            case WAIT_TRAJ1:
                if(capPos->getLastPosXY().distanceTo(traj[1].dest) < 0.5){
                    logs << INFO << "WAIT_TRAJ1" << traj[1].dest;
                    path.go2PointOrient(traj[2].dest, robot->env->obs, traj[2].angle);
                    stepPos = TRAJ2;
                }
                break;

            case TRAJ2:
                if(capPos->getLastPosXY().distanceTo(traj[2].dest) < 4.){
                    logs << INFO << "TRAJ2";
                    timeSave = millis();
                    stepPos = WAIT_TRAJ2;
                }
                break;

            case WAIT_TRAJ2: //attente sure de la position
                if(millis() - timeSave > 0){
                    logs << INFO << "WAIT_TRAJ2";
                    path.stopRobot(true);
                    timeSave = millis();
                    stepPos = WAIT_STOP1;
                }

                break;

            case WAIT_STOP1:
                if(millis() - timeSave > 1000){ //recale x
                    logs << INFO << "WAIT_STOP1";
                    Point2D<float> pos = capPos->getLastPosXY();
                    pos.x = traj[2].dest.x-10.;

                    sendMixPosPrimary(pos, traj[2].angle, MINVARIANCE_XY, MAXVARIANCE_XY, 0., 2*MINVARIANCE_THETA);

                    stepPos = WAIT_POS;
                }
                break;

            case WAIT_POS:
                if(capPos->getLastPosXY().x && capPos->getLastPosXY().y){
                    logs << INFO << "WAIT_POS" << capPos->getLastPosXY();
                    path.go2PointOrient(traj[3].dest, robot->env->obs, traj[3].angle);
                    stepPos = WAIT_TRAJ3;
                }
                break;

            case WAIT_TRAJ3: //attendre revenir entrer x
                if(capPos->getLastPosXY().distanceTo(traj[3].dest) < 0.5){
                    logs << INFO << "WAIT_TRAJ3" << traj[3].dest;
                    path.go2PointOrient(traj[4].dest, robot->env->obs, traj[4].angle);
                    stepPos = TRAJ4;
                }
                break;

            case TRAJ4: //attendre entrer y
                if(capPos->getLastPosXY().distanceTo(traj[4].dest) < 0.5){
                    logs << INFO << "TRAJ4";
                    path.go2PointOrient(traj[5].dest, robot->env->obs, traj[5].angle);
                    stepPos = TRAJ5;
                }
                break;

            case TRAJ5:
                if(capPos->getLastPosXY().distanceTo(traj[5].dest) < 4.){
                    logs << INFO << "TRAJ5";
                    timeSave = millis();
                    stepPos = WAIT_TRAJ5;
                }
                break;

            case WAIT_TRAJ5: //attente sure de la position
                if(millis() - timeSave > 0){
                    logs << INFO << "WAIT_TRAJ5";
                    path.stopRobot(true);
                    timeSave = millis();
                    stepPos = WAIT_STOP2;
                }

                break;

             case WAIT_STOP2:
                 if(millis() - timeSave > 1000){ //recale y
                     logs << INFO << "WAIT_STOP2";
                     Point2D<float> pos = capPos->getLastPosXY();
                     pos.y = traj[5].dest.y+10.;

                     sendMixPosPrimary(pos, traj[5].angle, MAXVARIANCE_XY, MINVARIANCE_XY, 0., MINVARIANCE_THETA);

                     stepPos = WAIT_POS2;
                 }
                 break;

             case WAIT_POS2:
                 if(capPos->getLastPosXY().x && capPos->getLastPosXY().y){
                     logs << INFO << "WAIT_POS2" << capPos->getLastPosXY();
                     path.go2PointOrient(traj[6].dest, robot->env->obs, traj[6].angle);
                     stepPos = POS_INIT;
                 }
                 break;

             case POS_INIT:
                 if(capPos->getLastPosXY().distanceTo(traj[6].dest) < 0.5){
                     path.go2PointOrient(traj[7].dest, robot->env->obs, traj[7].angle);
                     stepPos = WAIT_POS_INIT;
                 }
                 break;

             case WAIT_POS_INIT:
                 if(capPos->getLastPosXY().distanceTo(traj[7].dest) < 0.5){
                    _step = Step::WAIT_START;
                 }
                 break;

            default:
                break;

        }

*/
/*
























        if(wait){

            if(millis() - timeSave > 2000){
                wait = false;
             //   logs << ERR << "getchar";
  //  getchar();

//


                if(traj[index].action == 1){

                    Point2D<float> pos = capPos->getLastPosXY();
                    pos.x = traj[index].dest.x-10.;
                    path.stopRobot(true);
                    sendMixPosPrimary(pos, traj[index].angle, MINVARIANCE_XY, MAXVARIANCE_XY, 0., MINVARIANCE_THETA);
                    //recale x
                }
                else if(traj[index].action == 2){
                    path.stopRobot(true);
                    Point2D<float> pos = capPos->getLastPosXY();
                    pos.y = traj[index].dest.y+10.;
                  //  path.stopRobot(true);
                    sendMixPosPrimary(pos, traj[index].angle, MINVARIANCE_XY, MAXVARIANCE_XY, -M_PI/2, MINVARIANCE_THETA);
                    //recale y
                }
                //Wait new position was send
                pos = true;
                break;
            }
        }
        else if (pos){
            logs << INFO << "waiting..";

index++;
            if(capPos->getLastPosXY().x && capPos->getLastPosXY().y){

                pos = false;
                logs << INFO << "pos" << capPos->getLastPosXY();
                robot->env->obs[0].c.x = capPos->getLastPosXY().x;
                robot->env->obs[0].c.y = capPos->getLastPosXY().y;
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
                else if(traj[index].action == 1){
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
                else  if(traj[index].action == 5){
                    wait = true;
                    timeSave = millis();

                }

                //Wait new position was send
                index++;
                path.go2PointOrient(traj[index].dest, robot->env->obs, traj[index].angle);

            }
        }
*/
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
