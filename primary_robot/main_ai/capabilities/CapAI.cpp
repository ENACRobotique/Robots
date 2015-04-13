/*
 * CapAI.cpp
 *
 *  Created on: 13 avr. 2015
 *      Author: seb
 */

#include "CapAI.h"

#include "ai_types.h"
#include "tools.h"
#include "ai_tools.h"
#include "obj_tools.h"
#include "communications.h"
#include "ai.h"
extern "C"{
#include "millis.h"
}


int CapAI::loop(){
    static estate_t state = COLOR_SELECTION;
          static bool mode_obj = false;
          static int current_obj = -1;
          static Point2D<float> pt_select;
          static unsigned int last_time = 0;

          switch (state) {
              case COLOR_SELECTION: //Choose the color and take off the starting cord
                  startColor();
                  if (test_tirette()) {
                      float theta_robot;

                      if (robot->color == YELLOW) {
                          obs[0].c.x = INIT_POS_YELLOW_X;
                          obs[0].c.y = INIT_POS_YELLOW_Y;
                          theta_robot = INIT_ANGLE_YELLOW;
                      }
                      else if (robot->color == GREEN) {
                          obs[0].c.x = INIT_POS_GREEN_X;
                          obs[0].c.y = INIT_POS_GREEN_Y;
                          theta_robot = INIT_ANGLE_GREEN;
                      }
                      else {
                          cerr << "[ERROR] [ai.cpp] Error selection color" << endl;
                          return -1;
                      }

                      initObjective(robot->color);

                      Point2D<float> p(obs[0].c.x, obs[0].c.y);
                      sendPos(p, theta_robot); //Sending approximate initial position

                      //TODO procedure de mise en place

                      state = WAIT_STARTING_CORD;
                  }
                  break; //FIXME WAIT_POS delete because the ia doesn't known if abs pos is active or not, prop give the pos (or status)

              case WAIT_STARTING_CORD: //Wait to take in the starting cord
      #if SIMU
                  state = WAIT_START;
      #else
                  if(!test_tirette()) {
                      state = WAIT_START;
                  }
      #endif
                  break;

              case WAIT_START: //Wait the start (take off the starting cord)
                  if (test_tirette()) {
                      state = WAIT_SECONDARY;
                      _start_time = millis();
                      last_time = _start_time;
                  }
                  break;

              case WAIT_SECONDARY: //Waiting the secondary unblock the path
                  state = GAME;
                  break;

              case GAME: //Let's go
                  if (millis() - _start_time > END_MATCH){
                      state = SHUT_DOWN;
                      break;
                  }

                  if (!mode_obj) {
                      //Test if all objective have finished
                      if(listObj.empty())
                          logs << INFO << "Objective list is empty";

                      //Calculation of the next objective
                      if ((millis() - last_time) > 1000) {
                          last_time = millis();

                          if ((current_obj = next_obj()) != -1) {
                              pt_select = listObj[current_obj]->getDestPoint();
                              logs << INFO << "Selected point is (" << pt_select.x << " ; " << pt_select.y << ")";
      #if !HOLONOMIC
                        /*      int k = listObj[current_obj]->_EP;
                              sObjPt_t ep = listObj[current_obj]->entryPoint(k);
                              updateEndTraj(ep.angleEP,  &ep.c, ep.radiusEP);
                              printEndTraj();*/
      #endif
                              sPath_t path_loc = listObj[current_obj]->getPath();
                              path.addPath2(path_loc);
                              path.sendRobot();
                          }
                      }

                      //Test if the robot is on the entry point selected
                      Point2D<float> pos_robot = statuses.getLastPosXY(ELT_PRIMARY);
                      if ( (fabs(pt_select.x - pos_robot.x) < RESO_POS && fabs(pt_select.y - pos_robot.y) < RESO_POS) && (current_obj != -1) ){
                          mode_obj = true;
                      }
                  } else{
                      if (metObj(current_obj) == 0){
                          pt_select.x = -1;
                          pt_select.y = -1;
                          mode_obj = false;
                      }
                  }

                  // Robots simulation
                  if ((millis() - _start_time) > 2000) {
                       simuSecondary();
                  }

                  break;

              case SHUT_DOWN:
                  logs << INFO << "SHUT_DOWN : time = " << (unsigned int) (millis() - _start_time) / 1000;

                  path.stopRobot();

                  return 0;
                  break;

              default:
                  logs << ERR << "Unknown state=" << state;
                  break;
          }

          return 1;
}
