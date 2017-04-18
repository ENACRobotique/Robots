/*
 * motor.cpp
 *
 *  Created on: 6 mars 2017
 *      Author: fabien
 */
#include "motor.h"

double currentTime = 0;
double speed = MAX_SPEED;
double t1 = 0;
double t2 = 0;
double tFinal = 0;
long x1 = 0, x2 = 0, xTarget = 0, x1Real, x2Real, x = 0;
long thetaTarget = 0;
long baseIncRight = 0, baseIncLeft = 0, baseL = 0;
int sign = 0;

void initMotors() {
	pinMode(DIR_LEFT, OUTPUT);
	pinMode(DIR_RIGHT, OUTPUT);
	pinMode(PWM_LEFT, OUTPUT);
	pinMode(PWM_RIGHT, OUTPUT);
	analogWriteFrequency(PWM_LEFT, 20000);
	analogWriteFrequency(PWM_RIGHT, 20000);
	analogWriteResolution(10);
	analogWrite(PWM_LEFT, 0);
	analogWrite(PWM_RIGHT, 0);
}


void computeTrajParameters(long targetLength, long targetTheta) {

    baseL = getL();
    baseIncLeft = getIncLeft();
    baseIncRight = getIncRight();
    xTarget = abs(targetLength);
    thetaTarget = targetTheta;
    if(targetLength > 0) {
        sign = 1;
    } else {
        sign = -1;
    }
    //speed = min(MAX_SPEED, ((float)targetLength) / ACCEL);
    speed = MAX_SPEED;
    //t1 = ((float)speed)/ ACCEL;
    //t2 = targetLength/speed - ((float)speed)/ACCEL;
    //tFinal = t2 + t1;
    x1 = pow(speed,2) / (2*ACCEL);
    x2 = xTarget - x1;
    currentTime = 0;
    x = 0;
    /*speed = min(MAX_SPEED, targetLength / ACCEL);
    t1 = speed / ACCEL;
    t2 = targetLength/speed - speed/ACCEL;
    x1 = pow(speed,2) / (2*ACCEL);
    x2 = targetLength - x1;*/
}

int getConsigneLength(double t) {
    if(x < x1) {             //accélération
        //serial.printf("accel\n\r");
        x = (ACCEL * pow(t,2)) / 2;
        t1 = t;
        x1Real = x;
        return x;
    } else if (x < x2) {    //palier de vitesse
        //serial.printf("cruise\n\r");
        x = speed*(t-t1) + x1Real;
        x2Real = x;
        t2 = t;
        return x;
    } else {             //décélération
        if(x >= xTarget) {
            return xTarget;
        } else {
            int newX = x2Real + speed*(t-t2) - ((ACCEL * pow((t-t2),2)) / 2);
            if(newX < x) {
                return xTarget;
            } else {
                x = newX;
                return x;
            }
        }
    }
}

int getConsigneTheta(int xCons, double t) {
    if(xTarget != 0) {
        int theta = thetaTarget * ((float)xCons/xTarget);
        return theta;
    } else {
        /*if(t< t1) {               //accélération
                int x = (ACCEL * pow(t,2)) / 2;
                return x;
            } else if (t < t2) {    //palier de vitesse
                int x = speed*t + x1;
                return x;
            } else {                //décélération
                int x = x2 + speed*t - ((ACCEL * pow(t,2)) / 2);
                return x;
            }*/
        return 0;
    }
}


void controlMotors(long L, long nbIncLeft, long nbIncRight, int speedLeft, int speedRight, double dt) {
	static int ecartDistInt = 0;
	static int ecartOrientInt = 0;
    currentTime += dt;
    long incLeft = nbIncLeft - baseIncLeft;
    long incRight = nbIncRight - baseIncRight;
    long len = L - baseL;
    int speedRobot = (speedLeft + speedRight)/2;
    int xCons = sign * getConsigneLength(currentTime);
    int ecart = xCons - len;

    /*if(abs(ecart) < 100) {
        ecart = 0;
    }*/

    ecartDistInt += ecart;
    //ecartDistInt = CLAMP(-255, ecartDistInt, 255);
    double commandeDistance = ecart * KP_DIST + ecartDistInt*KI_DIST - KD_DIST * speedRobot;
    int orientation = incRight - incLeft;
    int orientationSpeed = speedRight - speedLeft;
    int consTheta = getConsigneTheta(xCons, currentTime);
    int ecartOrientation = consTheta - orientation;
    ecartOrientInt += ecartOrientation;
    //ecartOrientInt = CLAMP(-255, ecartOrientInt, 255);
    double commandeOrientation = ecartOrientation * 0.3 + ecartOrientInt*KI_ORIENT - KD_ORIENT * orientationSpeed;
    //commandeOrientation = CLAMP(0, commandeOrientation, 10);

    double rightCommand = commandeDistance + commandeOrientation;
    double leftCommand = commandeDistance - commandeOrientation;

    if(abs(rightCommand) < MIN_PWM) {
        rightCommand = 0;
    }
    if(abs(leftCommand) < MIN_PWM) {
            leftCommand = 0;
        }

    int absRightCommand = min(abs(rightCommand), 1023);
    int absLeftCommand  = min(abs(leftCommand), 1023);

    analogWrite(PWM_LEFT, absLeftCommand);
    analogWrite(PWM_RIGHT, absRightCommand);
    digitalWrite(DIR_LEFT, leftCommand > 0);
    digitalWrite(DIR_RIGHT, rightCommand < 0);

    /*Serial.printf("ecartOrient=%ld\torientation=%ld\tcommandOrient=", ecartOrientation, orientation);
    Serial.print(commandeOrientation);
    Serial.print("    right=");
    Serial.print(rightCommand);
    Serial.print("    left=");
    Serial.print(leftCommand);
*/
}
