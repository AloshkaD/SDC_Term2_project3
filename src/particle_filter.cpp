/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <stdio.h>
#include "particle_filter.h"
using namespace std;
const double PI  =3.141592653589793238463;
default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    const int m=100; // number of particles
	x = 4983;
	y = 5029;
	theta = 1.201;
	std_x = 2;
	std_y = 2;
	std_theta = 0.05;
	
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_x(y, std[1]);
	normal_distribution<double> dist_x(theta, std_[2]);
    weight_val = 1.0;
	for (int i = 0; i < m; ++i){
		double sample_x = dist_x(gen);
		double sample_y = dist_y(gen);
		double sample_theta = dist_theta(gen);
		//std.append[sample_x, sample_y, sample_theta ]
		
        particles.push_back(Particle{i, sample_x, sample_y, sample_theta, weight_val});
        weights.push_back(weight_val);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	

	/*
	// This is th'e main equation th'at Ive translated from C++ to Pyth'on'
	x_init = 102
	y_init= 65
	theta_init= (5*PI)/8
	v=110
	phai = PI/8
	dt=0.1
	
	div = velocity/ yaw_rate;
	x_f = x_init + div*(math.sin(theta_init + phai*dt)- math.sin(theta_init)); 
	y_f = y_init + div*(math.cos(theta_init) - math.cos(theta_init + phai*dt)); 
	phai_f = theta_init+phai*dt; 
	*/
	double dt = delta_t;
	if (fabs(yaw_rate) <0.001){
		yaw_rate = 1.0;
	} 

		vdt = velocity * dt;
		ydt = yaw_rate * dt;
        
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
 
 
 