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

#define MSG_DOWN_MAX_SIZE 47
#define DOWN_HEADER_SIZE 3 //Number of octet not taken into account for the checksum computation
#define MSG_UP_MAX_SIZE 9

#define BAUDRATE 115200

#define HWSERIAL Serial1 //The serial of the teensy used to link with the raspi



/////// THE FOLLOWING DEFINES MESSAGES RASPI -> TEENSY (Down messages)////////

typedef enum{TRAJECTOIRE, STOP, RESTART, RECALAGE, EMPTY_POINTS, DO_RECALAGE, RESET,
//2017 Specials :
    START_BALL_PICKER_MOTOR,
    STOP_BALL_PICKER_MOTOR,
    START_CANNON_MOTOR,
    STOP_CANNON_MOTOR,
    OPEN_CANNON_BARRIER,
    CLOSE_CANNON_BARRIER,
    OPEN_ROCKET_LAUNCHER,
    LOCK_ROCKET_LAUNCHER
}eTypeDown;



typedef struct __attribute__((packed)){

  uint8_t nb_trajectories; //must be <= 10 !

  uint8_t traj_speed;

  uint16_t theta_final; //<<13 for radians or <<7 for degrees

  struct __attribute__((packed)){

   uint16_t x; //mm

   uint16_t y; //mm

  }element[(MSG_DOWN_MAX_SIZE - DOWN_HEADER_SIZE - 4)/4];

}sTrajectory; //4 + n * 4 bytes



typedef struct __attribute__((packed)){

  uint16_t x; //mm

  uint16_t y; //mm

  uint16_t theta; //<<13 for radians or <<7 for degrees

}sRecalage; // 6 bytes



typedef struct __attribute__((packed)){

  uint8_t id; //message id for resend

  eTypeDown type :8; //type

  uint8_t checksum; //checksum !

  union __attribute__((packed)){

    sTrajectory traj;

    sRecalage recalage;

  };

}sMessageDown; // 3 + [4 + n*4] || [6]

////////// END RASPI -> TEENSY MESSAGES ////////
/////// THE FOLLOWING DEFINES MESSAGES TEENSY -> RASPI (Up messages)////////
typedef enum{ACK, NON_ACK, POINT_REACHED, POSITION, POINTS_BUFFER_FULL, RECALAGE_OK}eTypeUp;

typedef struct __attribute__((packed)){
	eTypeUp type :8; //type
	uint8_t down_id; //the number of the DOWN message (answer to ack, non ack, position or point reached
	uint16_t x; //X coord (for position and point_reached)
	uint16_t y; //Y coord (for position and point_reached)
	uint16_t theta; //Orientation of the robot (for position and point_reached) <<13 for radians or <<7 for degrees
	uint8_t point_id; //Point reached position in the list of the down message (so <= 14)
}sMessageUp; //9 bytes

/*Must be called once before checking or sending messages
 *
 * */
void message_init(int baudrate=BAUDRATE);

/* Read the data available on HWSERIAL and store fill the msg given in argument
 * (one message max per function call)
 * message_init() must have been called
 * Returns an int for check state purposes :
 * 1 : new message in msg !
 * 0 : no new message, msg unchanged
 * <0 : error
 * */
int message_recieve(sMessageDown * msg);

/* Send the message msg via HWSERIAL (must be type POINT_REACHED or POSITION)
 * message_init() must have been called
 * Returns an int for check state purposes
 * 0 : Msg sent
 * <0 : Error
 */
int message_send(sMessageUp msg);

#endif /* MESSAGES_H_ */
