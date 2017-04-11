/*
 * messages.h
 *
 *  Created on: 10 avr. 2017
 *      Author: guilhem
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_
#include "Arduino.h"
#include <avr/io.h>

#include <avr/interrupt.h>

#define MAX_SIZE 64

#define MSG_MAX_SIZE 60

#define BAUDRATE 115200

#define HWSERIAL Serial1 //The serial of the teensy used to link with the raspi



/////// THE FOLLOWING DEFINES MESSAGES RASPI -> TEENSY////////

typedef enum{TRAJECTOIRE, STOP, RESTART, RECALAGE}eType;



typedef struct __attribute__((packed)){

  uint8_t nb_trajectories; //must be <= 14 !

  uint8_t traj_speed;

  uint16_t theta_final; //<<13 for radians or <<7 for degrees

  struct __attribute__((packed)){

   uint16_t x; //mm

   uint16_t y; //mm

  }element[];

}sTrajectory; //4 + n * 4 bytes



typedef struct __attribute__((packed)){

  uint16_t x; //mm

  uint16_t y; //mm

  uint16_t theta; //<<13 for radians or <<7 for degrees

}sRecalage; // 6 bytes



typedef struct __attribute__((packed)){

  uint8_t id; //message id for resend

  eType type :8; //type

  uint8_t checksum; //checksum !

  union __attribute__((packed)){

    sTrajectory traj;

    sRecalage recalage;

  };

}sMessage; // 3 + [4 + n*4] || [6]

////////// END RASPI -> TEENSY MESSAGES ////////


/* Read the data available on SERIAL and store fill the msg given in argument
 * (one message max per function call)
 * Returns an int for check state purposes :
 * 1 : new message in msg !
 * 0 : no new message, msg unchanged
 * <0 : error
 * */
int message_recieve(sMessage * msg);



#endif /* MESSAGES_H_ */
