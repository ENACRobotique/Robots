#if defined(ARDUINO) && ARDUINO > 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <arm_hrrr/msgServosArm_hrrr.h>

#define NB_SERVOS 4
#define VREF 3.3
#define STEP2V VREF/1023.
#define VPOT_MIN 0.58
#define VPOT_MAX 2.5
#define FACT 1000
#define Pi 3.14159265359


// _____ About the joints _____
typedef struct{
  Servo s;
  String name;
  int pinCmd;
  int pinRead;
}smartServo;
String servoNames[NB_SERVOS] = {"_J0", "_J1", "_J2", "_J3"};
int servoPinsCmd[NB_SERVOS] = {3, 5, 6, 9};
int servoPinsRead[NB_SERVOS] = {4, 1, 2, 3};
int servoInitCmds[NB_SERVOS] = {90, 15, 177, 135};
int servoMins[NB_SERVOS] = {665, 700, 680, 610};
int servoMaxs[NB_SERVOS] = {2335, 1970, 2430, 2330};
int servoMinsCmd[NB_SERVOS] = {Pi/2.*FACT, 0*FACT, -Pi*FACT, -(3.*Pi*FACT)/4.};
int servoMaxsCmd[NB_SERVOS] = {-Pi/2.*FACT, (3.*Pi*FACT)/4., 0*FACT, (Pi*FACT)/4.};
int servoVPOT_MIN[NB_SERVOS] = {0.59*FACT, 0.59*FACT, 0.59*FACT, 0.59*FACT};
int servoVPOT_MAX[NB_SERVOS] = {2.5*FACT, 2.5*FACT, 2.5*FACT, 2.5*FACT};
//int servoDir[NB_SERVOS] = {1, 1, -1, 1};
smartServo servos[NB_SERVOS];

// _____ Debug _____
const int pinLed = 13;

// _____ ROS node and topics_____
ros::NodeHandle nh;
arm_hrrr::msgServosArm_hrrr js; 
ros::Publisher chatter_posServos("joint_state_Arduino", &js);

// _____ FUNCTIONS _____
void servoCmd_cb(const arm_hrrr::msgServosArm_hrrr& cmd_msg){
  for(int i=0; i<NB_SERVOS; i++){
    int v = map(cmd_msg.data[i]*FACT, 
          servoMinsCmd[i], servoMaxsCmd[i], servoMins[i], servoMaxs[i]);
    int v2 = constrain(v, servoMins[i], servoMaxs[i]);
    servos[i].s.writeMicroseconds(v2);
    char log_msg[64];
    sprintf(log_msg, "JCA:\n"
        "\t cmd = %d, appl = %d, const = %d", (int)(cmd_msg.data[i]*FACT),
        v, v2);
    if(i==-1)
      nh.loginfo(log_msg); 
  }
}

void readServoPos(){
  int u0, u1;
  int u = 0;
  for(int i=0; i<NB_SERVOS; i++){
    u0 = analogRead(servoPinsRead[i]);
    u1 = u0*STEP2V;
    u = map(u0*STEP2V*FACT, 
          servoVPOT_MIN[i], servoVPOT_MAX[i],
          servoMinsCmd[i], servoMaxsCmd[i]);  // In degree
    double u2 = u/FACT;
    js.data[i] = u2; // In rad
    char log_msg[64];
    sprintf(log_msg, "JCA:\n"
        "\t read = %d, u1 = %d, map = %d, u2 = %d", 
        u0, u1, u, u2);
    if(i==0)
      nh.loginfo(log_msg);
  } 
}

void initServos(){
  for(int i=0; i<NB_SERVOS; i++)
    initServo(i, servoNames[i], servoPinsCmd[i], servoPinsRead[i]);
  for(int i=0; i<NB_SERVOS; i++){
    servos[i].s.write(servoInitCmds[i]);
    nh.loginfo("joint_controller_arduino: Sent inti cmd");  
  }
}

void initServo(int i, String name, int pinCmd, int pinRead){
  if(i>=0  &&  i<NB_SERVOS){
      servos[i].name = name;
      servos[i].pinCmd = pinCmd;  
      servos[i].pinRead = pinRead;
      
      servos[i].s.attach(pinCmd);
      
      pinMode(servoPinsRead[i], INPUT);
  }
 else
    nh.logerror("joint_controller_arduino: error during initServo"); 
} 


ros::Subscriber<arm_hrrr::msgServosArm_hrrr> sub_cmdServos("arm_goal_joint_states", servoCmd_cb);

void setup(){
  pinMode(pinLed, OUTPUT);
  analogReference(EXTERNAL);
  initServos();
  nh.initNode();
  nh.subscribe(sub_cmdServos);
  
  // Init chatter_posServos
  nh.advertise(chatter_posServos); 
}

void loop(){
  readServoPos();
  chatter_posServos.publish(&js);
  
  nh.spinOnce();
  delay(500);
}
