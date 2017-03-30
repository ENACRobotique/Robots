/*
 * Particle.cpp
 *
 *  Created on: 27 févr. 2017
 *      Author: guilhem
 */

#include "Particle.h"


using namespace std;

const Point2D<float> Particle::circleCenters[] {Point2D<float>(1150, 1960),
				Point2D<float>(1850, 1960),
				Point2D<float>(40, 650),
				Point2D<float>(2960, 650)};

Particle::Particle(const Particle &p){
	this->y = p.y;
	this->x = p.x;
	this->theta = p.theta;
	this->turnNoise = p.turnNoise;
	this->senseNoise = p.senseNoise;
	this->forwardNoise = p.forwardNoise;
	this->senseDistribution = normal_distribution<float>(0.0, senseNoise);
	this->forwardDistribution = normal_distribution<float>(0.0, forwardNoise);
	this->turnDistribution = normal_distribution<float>(0.0, turnNoise);
	random_device rd;
	generator = default_random_engine( rd() );
}

Particle::Particle(){
	random_device rd;
	generator = default_random_engine( rd() );
	this->randomFloat = uniform_real_distribution<float>(0.0, MAX_X);
	this->x = this->randomFloat(generator);
	this->randomFloat = uniform_real_distribution<float>(0.0, MAX_Y);
	this->y = this->randomFloat(generator);
	this->randomFloat = uniform_real_distribution<float>(0.0, M_PI * 2);
	this->theta = this->randomFloat(generator);
	this->randomFloat = uniform_real_distribution<float>(0.0, 100.0);
	this->turnNoise = this->randomFloat(generator);
	this->senseNoise = this->randomFloat(generator);
	this->forwardNoise = this->randomFloat(generator);
	this->senseDistribution = normal_distribution<float>(0.0, senseNoise);
	this->forwardDistribution = normal_distribution<float>(0.0, forwardNoise);
	this->turnDistribution = normal_distribution<float>(0.0, turnNoise);
}

Particle::Particle(float forwardNoise, float turnNoise, float senseNoise){
	random_device rd;
	generator = default_random_engine( rd() );
	this->randomFloat = uniform_real_distribution<float>(0.0, MAX_X);
	this->x = this->randomFloat(generator);
	this->randomFloat = uniform_real_distribution<float>(0.0, MAX_Y);
	this->y = this->randomFloat(generator);
	this->randomFloat = uniform_real_distribution<float>(0.0, M_PI * 2);
	this->theta = this->randomFloat(generator);
	this->randomFloat = uniform_real_distribution<float>(0.0, 20.0);
	this->turnNoise = forwardNoise;
	this->senseNoise = senseNoise;
	this->forwardNoise = forwardNoise;
	this->senseDistribution = normal_distribution<float>(0.0, senseNoise);
	this->forwardDistribution = normal_distribution<float>(0.0, forwardNoise);
	this->turnDistribution = normal_distribution<float>(0.0, turnNoise);
}

Particle::Particle(float x, float y, float theta, float forwardNoise, float turnNoise, float senseNoise) : x(x), y(y),
		theta(theta), forwardNoise(forwardNoise), turnNoise(turnNoise), senseNoise(senseNoise), forwardDistribution(normal_distribution<float>(0.0, forwardNoise)),
		turnDistribution(normal_distribution<float>(0.0, turnNoise)), senseDistribution(normal_distribution<float>(0.0, senseNoise)){
	random_device rd;
	generator = default_random_engine( rd() );
}

Particle::~Particle(){
	//TODO
}

vector<float> Particle::sense() const{

	vector<float> toRet(LIDAR_POINTS_NUMBER, -1.);

	/*for (int i = 0; i<CIRCLES_NUMBERS; i++){
		alpha = fmod(atan2(circleCenters[i].y - y, circleCenters[i].x - x) - theta * M_PI / 180., 2 * M_PI);
		toRet[round(alpha)] = sqrt(pow(circleCenters[i].y - y, 2) + pow(circleCenters[i].x - x, 2)) - CIRCLES_RADIUS + senseDistribution(generator);
	}
	for (int i = 0; i<LIDAR_POINTS_NUMBER; i++){
		if (toRet[i] < 0.){
			toRet[i] = MAX_LIDAR_VALUE;
		}
	}*/


	float a, b, c, delta, sqrtDelta, x1, y1, alpha, talpha, talpha2, xsol1, ysol1, xsol2, ysol2, d1, d2;
	float minMeasure;
	for (int i = 0; i < LIDAR_POINTS_NUMBER; i++){
		minMeasure = MAX_LIDAR_VALUE;
		alpha = i;
		talpha = tan(alpha);
		talpha2 = pow(talpha, 2);


		if (i == 90 ||i == 270){
			for (int j = 0; j < CIRCLES_NUMBERS; j++){
				x1 = circleCenters[j].x;
				y1 = circleCenters[j].y;
				a = 1;
				b = -2 * y1;
				c = pow(this->x - x1, 2) + pow(y1, 2) - CIRCLES_RADIUS_2;
				delta = pow(b, 2) - 4 * a *c;
				if (delta <= 0){
					minMeasure = min(minMeasure, MAX_LIDAR_VALUE);
				}else{
					ysol1 = (-b + sqrt(delta)) / (2 * a);
					ysol2 = (-b - sqrt(delta)) / (2 * a);
					d1 = min(abs(ysol1 - this->y), abs(ysol2 - this->y));
					minMeasure = min(minMeasure, d1);
				}
			}
		}else{
			for (int j = 0; j < CIRCLES_NUMBERS; j++){
				x1 = circleCenters[j].x;
				y1 = circleCenters[j].y;
				a = 1 + talpha2;
				b = -2 * x1 + 2 * talpha * (this->y - y1 - talpha * this->y);
				c = pow(x1,2) + pow(this->y - y1 - talpha * this->x, 2) - CIRCLES_RADIUS_2;
				delta = pow(b, 2) - 4 * a *c;
				if (delta <= 0){
					minMeasure = min(minMeasure, MAX_LIDAR_VALUE);
				}else{
					xsol1 = (-b + sqrt(delta)) / (2 * a);
					xsol2 = (-b - sqrt(delta)) / (2 * a);
					ysol1 = talpha * (xsol1 - this->x) + this->y;
					ysol2 = talpha * (xsol2 - this->x) + this->y;
					d1 = sqrt(pow(xsol1 - this->x, 2) + pow(ysol1 - this->y, 2));
					d2 = sqrt(pow(xsol2 - this->x, 2) + pow(ysol2 - this->y, 2));
					minMeasure = min(minMeasure, min(d1, d2));
				}
			}
		}
		toRet.push_back(minMeasure);
	}
	return toRet;
}

void Particle::move(float theta, float distance){

	float forwardError = forwardDistribution(generator);

	this->theta = fmod(this->theta + theta + senseDistribution(generator), 2 * M_PI);
	this->x = max(min(this->x + (distance + forwardError) * cos(theta), (float)MAX_X), 0.0f);
	this->y = max(min(this->y + (distance + forwardError) * sin(theta), (float)MAX_Y), 0.0f);
}


const double Particle::measurementProb(vector<float> measure){
	double prob = 1.0;
	vector<float> theoMeasurments = sense();
	for (int i = 0; i < LIDAR_POINTS_NUMBER; i++){
		prob *= Gaussian(theoMeasurments[i], this->senseNoise, measure[i]);
	}
	return prob;
}

const double Particle::Gaussian(double mu, double sigma, double x){
	return exp(- pow((mu - x), 2) / pow(sigma, 2) / 2.0) / sqrt(2.0 * M_PI * pow(sigma, 2));
}
