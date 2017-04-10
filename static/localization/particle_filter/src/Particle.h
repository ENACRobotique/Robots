/*
 * Particle.h
 *
 *  Created on: 23 févr. 2017
 *      Author: guilhem
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

#define MAX_X 3000.
#define MAX_Y 2000.
#define ANGLE_TO_LIDAR_ORIGIN 0.0
#define LIDAR_POINTS_NUMBER 360
#define CIRCLES_NUMBERS 4
#define CIRCLES_RADIUS 40
#define CIRCLES_RADIUS_2 1600
#define FLOAT_PRECISON 0.00000001
#define MAX_LIDAR_VALUE  5000.f

#include "Point2D.h"
#include <ctime>
#include <cstdlib>
#include <math.h>
#include <random>
#include <algorithm>

class Particle{

	public :
		float x;
		float y;
		float theta;
		float forwardNoise;
		float turnNoise;
		float senseNoise;
		
	public :
		static const Point2D<float> circleCenters[];
		Particle();
		Particle(const Particle &p);
		Particle(float forwardNoise, float turnNoise, float senseNoise);
		Particle(float x, float y, float theta, float forwardNoise, float turnNoise, float senseNoise);
		virtual ~Particle();
		std::vector<float> sense() const;
		void move(float theta, float distance);
		double measurementProb(std::vector<float> measure);
		static const double Gaussian(double mu, double sigma, double x);

	private:
		std::default_random_engine generator;
		std::uniform_real_distribution<float> randomFloat;
		std::normal_distribution<float> forwardDistribution;
		std::normal_distribution<float> turnDistribution;
		std::normal_distribution<float> senseDistribution;

};





#endif /* PARTICLE_H_ */
