/*
 * particle_filter.cpp
 * I aknowledge dolaameng for this code followes a similar structure as h'is code found at https://github.com/dolaameng/CarND-Kidnapped-Vehicle-Project/blob/master/src/particle_filter.cpp#L63 
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
//double  & g = gen;
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    const int m=100; // number of particles
	//double x = 4983;
	//double y = 5029;
	//float theta = 1.201;
	//double std_x = 2;
	//double std_y = 2;
	//double std_theta = 0.05;
	
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
    
	for (int i = 0; i < m; ++i){
		double sample_x = dist_x(gen);
		double sample_y = dist_y(gen);
		double sample_theta = dist_theta(gen);
		//std.append[sample_x, sample_y, sample_theta ]
		double weight_val = 1.0/m;
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
	auto  & g = gen; 
	double dt = delta_t;
	//if (fabs(yaw_rate) <0.001){
	//	double rotation = 0; ///avoid division by zero
//	} else {
	//	double rotation = velocity/ yaw_rate;
//	}
    double rotation = (fabs(yaw_rate) < 0.001) ? 0.0 : (velocity / yaw_rate);
	double vdt = velocity * dt;
	double ydt = yaw_rate * dt;
    transform(
            particles.begin(), particles.end(), particles.begin(),
			[&g, std_pos, dt, rotation,yaw_rate, ydt, vdt] (Particle p){
			
			//if (fabs(yaw_rate) > 0.001){
			//	double theta = p.theta + ydt,
			//	//x_f = x_init + div*(math.sin(theta_init + phai*dt)- math.sin(theta_init)); 
				
			//		x = p.x + rotation * (sin(theta) - sin(p.theta)), 
			//	//y_f = y_init + div*(math.cos(theta_init) - math.cos(theta_init + phai*d
			//		y = p.y + rotation * (cos(p.theta) - cos(theta));	
			//}
			//else {
			//	double theta = p.theta + ydt,
			//		x = p.x + vdt * cos (p.theta),
			//		y = p.y + vdt * sin (p.theta);
			//}
            double theta = p.theta + ydt;
            double x = (fabs(yaw_rate) < 0.001) ? (p.x + vdt * cos (p.theta)) : (p.x + rotation * (sin(theta) - sin(p.theta)));
			double y = (fabs(yaw_rate) < 0.001) ? (p.y + vdt * sin (p.theta)) : (p.y + rotation * (cos(p.theta) - cos(theta)));
			normal_distribution<double> dist_x(x, std_pos[0]);
			normal_distribution<double> dist_y(y, std_pos[1]);
			normal_distribution<double> dist_theta(theta, std_pos[2]);

			return Particle {p.id, dist_x(gen),dist_y(gen),dist_theta(gen),p.weight };

			}

	);   
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (LandmarkObs & ob : observations) {
		double x = ob.x;
		double y = ob.y;
		vector<double> to_landmark_dists(predicted.size());
		transform(
			predicted.begin(), predicted.end(),
			to_landmark_dists.begin(),
			[x, y](const LandmarkObs & landmark) {
				return dist(x, y, landmark.x, landmark.y);
			}
		);
		double minele= std::min_element(to_landmark_dists.begin(), to_landmark_dists.end()) - to_landmark_dists.begin();
		ob.id = minele;
	}
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
	//   th'e'
	//   http://planning.cs.uiuc.edu/node99.html
		double P = PI * std_landmark[0] * std_landmark[1];
		for (Particle & p : particles){
				double x = p.x;
				double y = p.y;
				float theta = p.theta;
				
				vector<LandmarkObs> obs_in_map{observations.size()
				};
				transform(
					observations.begin(), observations.end(), obs_in_map.begin(),
					[x, y, theta](LandmarkObs ob){
						return LandmarkObs{ob.id, ob.x* cos(theta)- ob.y*sin(theta) + x, ob.x* sin(theta)+ ob.y*cos(theta) + y
						 };
					}
				);
		vector<LandmarkObs> predicted_obs;
		for (const auto & landmark : map_landmarks.landmark_list) {
			bool in_range = (dist(x, y, landmark.x_f, landmark.y_f) <= sensor_range);
			if (!in_range) continue;
			predicted_obs.push_back(LandmarkObs{landmark.id_i,landmark.x_f,landmark.y_f});
		}
		dataAssociation(predicted_obs, obs_in_map);
		double weight = 0;
		if (predicted_obs.empty()){
			weight = 0.000001;
		}
		else {
			weight =1;
			double var_x = std_landmark[0] * std_landmark[0];
			double var_y = std_landmark[1] * std_landmark[1];
			for (const auto & ob : obs_in_map) {
				double dx = ob.x - predicted_obs[ob.id].x;  
				double dy = ob.y - predicted_obs[ob.id].y;
				weight *= exp( -dx*dx/(2*var_x) - dy*dy/(2*var_y) ) / P;
			}
			
		} 

        p.weight = weight;
		weights[p.id] = weight; 
   }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    // auto  & g = gen;
	discrete_distribution<int> sample_dist(weights.begin(), weights.end());
	vector<Particle> new_particles(particles.size());
	for (int i = 0; i < particles.size(); ++i) {
		Particle & p = new_particles[i];
		p.id = i;
		p.weight = 1.;
		int point = sample_dist(gen);
		p.x = particles[point].x;
		p.y = particles[point].y;
		p.theta = particles[point].theta;
	}
	particles = new_particles;    
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
 
 
 