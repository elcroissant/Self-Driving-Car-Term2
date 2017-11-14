/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	num_particles = 100;
	particles.resize(num_particles);
	weights.resize(num_particles);
	for (int i =0; i<particles.size(); i++)
	{
		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta  = dist_theta(gen);
		particles[i].weight = 1.0;
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	const double theta_dot_t = yaw_rate * delta_t;
	const double velocity_f = (abs(yaw_rate) > 0.001) ? velocity/yaw_rate : velocity * delta_t;


	default_random_engine gen;

	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);


	for (int i=0; i< num_particles; i++)
	{
		double theta = particles[i].theta;
		double theta_f = theta + theta_dot_t;

		if (abs(yaw_rate) > 0.001)
		{
			particles[i].x += velocity_f * (sin(theta_f) - sin(theta)) + dist_x(gen);
		 	particles[i].y += velocity_f * (cos(theta) - cos(theta_f)) + dist_y(gen);
			particles[i].theta += theta_dot_t + dist_theta(gen);
		}
		else
		{
			particles[i].x += velocity_f * cos(theta) + dist_x(gen);
			particles[i].y += velocity_f + sin(theta) + dist_y(gen);
			//particles[i].theta += dist_theta(gen);
		}
	}
    
}

// Find the predicted measurement that is closest to each observed measurement and assign the 
// observed measurement to this particular landmark.
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {

	for (int i = 0; i < observations.size(); i++)
	{
		const double x_obs = observations[i].x;
		const double y_obs = observations[i].y;
		
		double minimal_dist = 0;

		for (int j=0; j < predicted.size(); j++)
		{
			const double x_pred = predicted[i].x;
			const double y_pred = predicted[i].y;
			const double distance = sqrtl(powl(x_obs - x_pred,2) + powl(y_obs - y_pred,2));

			// if id is 0 set initial value for minimal_dist as distance between observation and first predicted
			if (observations[i].id == 0 || ((observations[i].id != 0) && (distance < minimal_dist)) )
			{
				minimal_dist = distance;
				observations[i].id = predicted[i].id;
			}
		}
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	std::vector<LandmarkObs> map_observations;
	std::vector<LandmarkObs> landmarks_in_range;

	for (int i=0; i < num_particles; i++)
	{
		double x_part = particles[i].x;
		double y_part = particles[i].y;
		double theta = particles[i].theta;

		for (int j=0; j < observations.size(); j++)
		{
			double x_obs = observations[i].x;
			double y_obs = observations[i].y;

			// transform to map x coordinate
			double x_map = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
			// transform to map y coordinate
			double y_map = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

			map_observations.push_back(LandmarkObs { 0 /*init index = 0*/, x_map, y_map });
		}

		for(int i=0; i < map_landmarks.landmark_list.size(); i++)
		{
			float x_land = map_landmarks.landmark_list[i].x_f;
			float y_land = map_landmarks.landmark_list[i].y_f;
			int id = map_landmarks.landmark_list[i].id_i;

			float distance = sqrtf(powl(x_land - x_part,2) + powl(y_land - y_part,2));

			if (distance < sensor_range){
				landmarks_in_range.push_back( LandmarkObs { id, x_land, y_land });
			}
		}

		dataAssociation(landmarks_in_range, map_observations);
		
	
	
		const double denom_x = 2 * pow(std_landmark[0],2);
		const double denom_y = 2 * pow(std_landmark[1],2);
		// calculate normalization term
		const double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
		
		for (int i = 0; i < map_observations.size(); i++)
		{
			if (map_observations[i].id == 0)
			{
				continue;
			}
	
			double x_obs = map_observations[i].x;
			double y_obs = map_observations[i].y;
			double mu_x, mu_y = 0;

			int prediction_id = map_observations[i].id;
	
			for (int i=0; i< landmarks_in_range.size(); i++)
			{
				if (prediction_id == landmarks_in_range[i].id)
				{
					mu_x = landmarks_in_range[i].x;
					mu_y = landmarks_in_range[i].y;
				}
			}
			// calulate exponent
			double exponent= pow(x_obs - mu_x,2)/denom_x + pow(y_obs - mu_y,2)/denom_y;
			// calculate weight using normalization terms and exponent
			particles[i].weight *= gauss_norm * exp(-exponent);
		}
	
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
