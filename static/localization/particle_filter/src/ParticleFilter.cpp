/*
 * ParticleFilter.cpp
 *
 *  Created on: 18 mars 2017
 *      Author: guilhem
 */

#include "ParticleFilter.h"

using namespace std;

ParticleFilter::ParticleFilter() {
	for (int i = 0; i < PARTICLES_NUMBER; i++){
		particles[i] = Particle();
#ifdef DEBUG
		cout << particles[i].x << ":" << particles[i].y << "\n";
#endif
	}
	random_device rd;
	generator = default_random_engine( rd());
}

ParticleFilter::~ParticleFilter() {
	// TODO Auto-generated destructor stub
}

void ParticleFilter::move(float theta, float distance){
	for (int i = 0; i < PARTICLES_NUMBER; i++){
		this->particles[i].move(theta, distance);
	}
}

void ParticleFilter::sense(vector<float> measure){
	Particle resampledParticles[PARTICLES_NUMBER];
	double weights[PARTICLES_NUMBER];
	double maxWeight = 0.0;
	double beta = 0.0;
	int turningIndex = rand() % PARTICLES_NUMBER;
	uniform_real_distribution<double> distribution(0.0, 1.0); //Engine for random numbers [0.0; 1.0[
	for (int i = 0; i < PARTICLES_NUMBER; i++){
		weights[i] = particles[i].measurementProb(measure);
		maxWeight = max(weights[i], maxWeight);
		//cout << "weight : " << weights[i];
	}
	for (int i = 0; i < PARTICLES_NUMBER; i++){
		beta += distribution(generator) * 2.0 * maxWeight;
		while (beta > weights[turningIndex]){
			beta -= weights[turningIndex];
			turningIndex = (turningIndex + 1) % PARTICLES_NUMBER;
		}
		resampledParticles[i] = particles[turningIndex];
	}
	cout << "new\n";
	for (int i = 0; i < PARTICLES_NUMBER; i++){
		particles[i] = resampledParticles[i];
		cout << particles[i].x << ":" << particles[i].y << "\n";
		//cout << "Particle : " << i << "X : " << particles[i].x << "Y : " << particles[i].y << "\n";

	}
}

PointOrient2D<float> ParticleFilter::locate(){
	int selectedPaticle = rand() % PARTICLES_NUMBER;
	return PointOrient2D<float>(particles[selectedPaticle].x, particles[selectedPaticle].y,
			particles[selectedPaticle].theta);
}

PointOrient2D<float> ParticleFilter::locate(vector<float> measure){
	vector<double> weights = vector<double>(PARTICLES_NUMBER);
	int maxWeightIndex;
	for (int i = 0; i < PARTICLES_NUMBER; i++){
		weights[i] = particles[i].measurementProb(measure);
		//cout << "weight : " << weights[i];
	}
	maxWeightIndex = distance(weights.begin(), min_element(weights.begin(), weights.end()));
	return PointOrient2D<float>(particles[maxWeightIndex].x, particles[maxWeightIndex].y,
				particles[maxWeightIndex].theta);
}
