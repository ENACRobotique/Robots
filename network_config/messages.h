/*
 * messages.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 *
 *
 *      Remark concerning this file :
 *      The user MUST ONLY modify the content between the matching commented lines *** user XXXX start *** and *** user XXXX stop ***
 *      He/she MUST NOT modify anything outside these markers
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "../static/communication/botNet/shared/message_header.h"


// bn_Address : cf SUBNET_MASK and ADDRxx_MASK in network_cfg.h

#define BN_MAX_PDU 64 //max size of a message, including header AND payload.

//message types
typedef enum{
    E_DEBUG,                // general debug
    E_ROLE_SETUP,           // setup addresses of some standard nodes + automatic relaying of messages between them
    E_DATA,                 // arbitrary data payload
    E_ACK_RESPONSE,         // ack response
    E_PING,
    E_TRACEROUTE_REQUEST,   // traceroute request
    E_TRACEROUTE_RESPONSE,  // traceroute response
    E_INTP,                 // bn_intp synchronization message

/************************ user types start ************************/
    E_SWITCH_CHANNEL,       // switch channel message
    E_SYNC_DATA,            // sync data (send from the turret to the receiver)
    E_SYNC_OK,              // synced
    E_PERIOD,               // period measurement
    E_MEASURE,              // laser delta-time measurement
    E_TRAJ,                 // a trajectory step
    E_POS,                  // position (w/ uncertainty) of an element
    E_SERIAL_DUMP,          // serial dump (for debug)
    E_ASSERV_STATS,         // control loop statistics
    E_GOAL,                 // asks the robot to go to this goal (x,y)
    E_OBS_CFG,              // obstacle array config
    E_OBSS,                 // obstacles update (position & status update)
    E_POS_QUERY,            // position query
    E_SERVOS,               // servo messages
    E_IHM_STATUS,           // ihm status
    E_SPEED_SETPOINT,       // speed setpoint
    E_GENERIC_STATUS,       // generic status of an element
    E_POS_STATS,            // position statistics (packed)
/************************ user types stop ************************/

    E_TYPE_COUNT            // This one MUST be the last element of the enum
}E_TYPE;

/************************ !!! user action required start ************************/
/* WARNING : BN_TYPE_VERSION describes the version of the above enum.
 It is managed "by hand" by the user. It MUST be incremented (modulo 16) only when the order of the *already existing* elements
 of this enum is modified.
 Examples :
     New type AFTER the last one (but before E_TYPE_COUNT) : do not increment BN_TYPE_VERSION
     New type between 2 previously exinsting types : increment BN_TYPE_VERSION
     Cleaning (removing) unused type : increment BN_TYPE_VERSION
     Changing the order of the types : increment BN_TYPE_VERSION
 The recommended use is the following :
     Add every new type at the end of the list (but before E_TYPE_COUNT), do not delete old ones.
     Every month (or so, when it is needed) : remove unused types, sort logically the other types and increse BN_TYPE_VERSION.
 */
#define BN_TYPE_VERSION 0       //last modifed : 2013/10/31
/************************ user action required stop ************************/

//function returning a string corresponding to one element of the above enum. Must be managed by hand.
const char *eType2str(E_TYPE elem);

//Specific payloads
typedef struct __attribute__((packed)){
    uint8_t     index;  // index of the current message (n)
    uint32_t    time;  // date of sending of the current message (n) in the master's clock
    uint32_t    prevTime;  // date of sending of the previous message (n-1) in the master's clock
}sINTP;

/************************ user payload definition start ************************/
//user-defined payload types.
//for simple payloads ( single variable or array), this step can be skipped
//Warning : the user has to make sure that these payloads are not too big (cf BN_MAX_PDU)

typedef struct __attribute__((packed)){ // 2bytes
    struct __attribute__((packed)){
        uint8_t first  :4;
        uint8_t second :4;
    } sendTo;
    struct __attribute__((packed)){
        uint8_t n1 :4;
        uint8_t n2 :4;
    } relayTo;
} sRoleActions;
typedef struct __attribute__((packed)){
    uint16_t nb_steps; // must be <=13 to fit in a sMsg payload (2+4*13=54)
    struct{ // 4bytes
        enum{
            UPDATE_ADDRESS,
            UPDATE_ACTIONS
        } step :8;
        union{
            struct __attribute__((packed)){
                uint8_t role;
                bn_Address address;
            };
            struct __attribute__((packed)){
                E_TYPE type :8;
                sRoleActions actions;
            };
        };
    } steps[];
} sRoleSetupPayload;

typedef struct __attribute__((__packed__)){
    uint32_t value;          //laser measured distance (in mm)
    uint32_t date;           //laser sensing time
    uint16_t precision;      //precision of the measure
    uint16_t sureness;       //sureness of the mesure
} sMobileReportPayload;

typedef enum {
    SYNCF_BEGIN_ELECTION,
    SYNCF_MEASURES,
    SYNCF_END_MEASURES,
    SYNCF_OK
} syncFlag;

typedef struct __attribute__((__packed__)){
    uint32_t lastTurnDate;   //last turn date (in µs)
    uint32_t period;         //last measured period (instantaneous, measured at the same time as lastTurnDate, in µs)
    int16_t  index;          //index of the current
    syncFlag flag;
} sSyncPayload;

typedef struct {
// segment
    float p1_x;
    float p1_y;
    float p2_x;
    float p2_y;
    float seg_len;
// circle
    float c_x;
    float c_y;
    float c_r; // >0 CW | <0 CCW
    float arc_len;
// trajectory data
    uint16_t tid; // trajectory identifier
    uint16_t sid; // step identifier
} sTrajElRaw_t;

typedef struct {
// position in game area frame
    float x; // (cm)
    float y; // (cm)
    float theta; // (rad)
// uncertainty (oriented rectangle)
    float u_a_theta; // (rad)
    float u_a; // (cm)
    float u_b; // (cm)
// trajectory steps
    int tid; // trajectory identifier               // FIXME int -> intxx_t
    int sid; // step identifier                     // FIXME int -> intxx_t
    uint8_t ssid; // sub-step identifier (0:line, 1:circle)
// identifier of the robot/element
    uint8_t id; // 0:prim, 1:sec, 2:adv_prim, 3:adv_sec
} sPosPayload; // TODO rename to E_PROP_STATUS, contains an E_GENERIC_POS

typedef enum{
    ELT_PRIMARY,
    ELT_SECONDARY,
    ELT_ADV_PRIMARY,
    ELT_ADV_SECONDARY,
    ELT_FIRE,

    NUM_E_ELEMENT
} eElement;

typedef enum{
    SERVO_PRIM_DOOR,
    SERVO_PRIM_FIRE1,
    SERVO_PRIM_FIRE2,
    SERVO_PRIM_ARM_LEFT,
    SERVO_PRIM_ARM_RIGHT,

    NUM_E_SERVO
} eServos;

typedef enum{
    IHM_STARTING_CORD,
    IHM_MODE_SWICTH,
    IHM_LED
} eIhmElement;

typedef struct __attribute__((packed)){
// position in frame (specified in field "frame")
    float x;            // (cm)
    float y;            // (cm)
    float theta;        // (rad)
    enum{
        FRAME_PLAYGROUND,
        FRAME_PRIMARY
    } frame :8;
} s2DPosAtt; // 2D position & attitude

typedef struct __attribute__((packed)){
// uncertainty (oriented 2D ellipse)
    float theta;      // uncertainty of the 2D orientation (rad)
    float a_angle;    // orientation of the uncertainty ellipse along "a" axis (rad)
    float a_std;      // standard deviation along "a" axis (cm)
    float b_std;      // standard deviation along "b" axis (cm)
} s2DPAUncert;

typedef struct __attribute__((packed)){
    uint32_t date;      // synchronized date (µs)
    eElement id :8;
    union{
        // generic way to access position (if present in type)
        struct{
            s2DPosAtt pos;
            s2DPAUncert pos_u;
        };

        // in case of pos.id == ELT_PRIMARY
        struct{
            s2DPosAtt pos;
            s2DPAUncert pos_u;

            float speed; // (cm/s)

            uint16_t tid; // trajectory identifier
            uint8_t sid; // step identifier
            uint8_t ssid; // sub-step identifier (0:line, 1:circle)
        } prop_status;

        // in case of pos.id == ELT_ADV_*
        struct{
            s2DPosAtt pos;
            s2DPAUncert pos_u;
        } adv_status;

        // in case of pos.id == ELT_FIRE
        struct{
            s2DPosAtt pos;
            s2DPAUncert pos_u;

            enum{
                FIRE_HORIZ_YELLOW,
                FIRE_HORIZ_RED,
                FIRE_VERTICAL,
                FIRE_OBLIQUE,
                FIRE_VERTICAL_TORCH
            } state :8;
        } fire_status;
    };
} sGenericStatus;

typedef struct{
    uint32_t date; // synchronized date (µs)
    eElement id :8;
} sPosQuery;

typedef struct __attribute__((packed)){
    uint8_t nb_obs;
    float r_robot;
    float x_min, x_max, y_min, y_max;
} sObsConfig;

#define MAX_NB_OBSS_PER_MSG (6)
typedef struct __attribute__((packed)){
    uint16_t nb_obs; // must be <= 6 (2 + 8*6 = 50)
    struct __attribute__((packed)){ // 8bytes
        int16_t x; // (LSB=0.1mm)
        int16_t y; // (LSB=0.1mm)
        int16_t r; // (LSB=0.1mm)

        uint8_t moved :4;
        uint8_t active :4;
        uint8_t id; // index in the tab of obstacles
    } obs[];
} sObss;

#define NB_ASSERV_STEPS_PER_MSG (4)
typedef struct __attribute__((packed)){
    uint16_t nb_seq;
    struct __attribute__((packed)){ // 13bytes*4
        unsigned short delta_t;
        short ticks_l;
        short ticks_r;
        short consigne_l;
        short consigne_r;
        short out_l :12;
        short out_r :12;
    } steps[NB_ASSERV_STEPS_PER_MSG];
} sAsservStats;

#define NB_POS_STEPS_PER_MSG (7)
typedef struct __attribute__((packed)){
    uint16_t nb_seq;
    struct __attribute__((packed)){ // 7bytes
        unsigned short delta_t; // (µs)
        uint16_t x :14; // 0-16383 (quarter of mm)
        uint16_t y :14; // 0-16383 (quarter of mm)
        uint16_t theta :12; // 0-4095 (tenth of °)
    }steps[NB_POS_STEPS_PER_MSG];
} sPosStats;

typedef struct __attribute__((packed)){
    uint16_t nb_servos; // must be <=18
    struct __attribute__((packed)){
            eServos id :8; // identifier of the servomotor
            uint16_t us; // servo setpoint in µs
    } servos[];
} sServos;

typedef struct __attribute__((packed)){
    uint16_t nb_states; // must be <=18
    struct __attribute__((packed)){
        eIhmElement id :8; // identifier of the ihm element
        uint16_t state; // status
    } states[];
} sIhmStatus;

typedef struct{
    float speed;
} sSpeedSetPoint;
/************************ user payload definition stop ************************/





/*
 * union defining the different possible payloads
 */
typedef union{
    uint8_t raw[BN_MAX_PDU-sizeof(sGenericHeader)];		//only used to access data/data modification in low layer
    uint8_t data[BN_MAX_PDU-sizeof(sGenericHeader)];	//arbitrary data, actual size given by the "size" field of the header
    uint8_t debug[BN_MAX_PDU-sizeof(sGenericHeader)];   //debug string, actual size given by the "size" field of the header
    sAckPayload ack;
    sRoleSetupPayload roleSetup;
    sINTP   intp;

/************************ user payload start ************************/
//the user-defined payloads from above must be added here. The simple ones can be directly added here
//Warning : the user has to make sure that these payloads are not too big (cf BN_MAX_PDU)
    uint8_t channel;
    uint32_t period;
    sTrajElRaw_t traj;
    sPosPayload pos;
    sMobileReportPayload mobileReport;
    sSyncPayload sync;
    sAsservStats asservStats;
    sPosStats posStats;
    sObsConfig obsCfg;
    sObss obss;
    sGenericStatus genericStatus;
    sPosQuery posQuery;
    sServos servos;
    sIhmStatus ihmStatus;
    sSpeedSetPoint speedSetPoint;
/************************ user payload stop ************************/

}uPayload;


//final message structure
typedef struct{
    sGenericHeader header;
    uPayload payload;
}sMsg;

#ifdef __cplusplus
}
#endif

#endif /* MESSAGES_H_ */
