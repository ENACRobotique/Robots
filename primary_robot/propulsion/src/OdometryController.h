/*
 * Odometry.h
 *
 *  Created on: 27 avr. 2017
 *      Author: darian
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

class OdometryController {
public:
	OdometryController();
	virtual ~OdometryController();
	void init(double posXi, double posYi, double thetaRadi);
	void razIncs();
	void updatePosition();

	long getLength();

	/*incr/decr increments*/
	void ISRLeft();
	void ISRRight();

	/*Getters*/
	volatile int getNbIncLeft() const {
		return _nbIncLeft;
	}

	volatile int getNbIncRight() const {
		return _nbIncRight;
	}

	double getPosX() const {
		return _posX;
	}

	double getPosY() const {
		return _posY;
	}

	double getThetaRad() const {
		return _thetaRad;
	}

	long getPrevL() const {
		return _prevL;
	}

	long getLeftAcc() const {
		return _leftAcc;
	}

	long getRightAcc() const {
		return _rightAcc;
	}

	long getSpeedLeft() const {
		return _speedLeft;
	}

	long getSpeedRight() const {
		return _speedRight;
	}

private:
	double _posX;
	double _posY;
	double _thetaRad;
	volatile int _nbIncLeft;
	volatile int _nbIncRight;
	long _prevNbIncLeft;
	long _prevNbIncRight;
	long _prevL;
	long _leftAcc;
	long _rightAcc;
	long _speedLeft, _speedRight;


};

extern OdometryController Odometry;

#endif /* ODOMETRY_H_ */
