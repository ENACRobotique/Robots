/*
 * ParticleFilter.h
 *
 *  Created on: 18 mars 2017
 *      Author: guilhem
 */

#ifndef PARTICLEFILTER_H_
#define PARTICLEFILTER_H_

#include "PointOrient2D.h"
#include "Particle.h"
#include <vector>
#include <random>
#include <iostream>

#define DEBUG

#define PARTICLES_NUMBER 1000

class ParticleFilter {
public:
	ParticleFilter();
	virtual ~ParticleFilter();
	PointOrient2D<float> locate();
	PointOrient2D<float> locate(std::vector<float> measure);
	void move(float theta, float distance);
	void sense(std::vector<float> measure);


public:
	Particle particles[PARTICLES_NUMBER];
	std::default_random_engine generator;
};

#endif /* PARTICLEFILTER_H_ */
