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
    num_particles = 500;
    
    default_random_engine gen;
    
    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];
    
    // Create normal (Gaussian) distribution for x, y and theta;
    normal_distribution<double> dist_x(x, std_x);
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> heading(theta, std_theta);

    for(int i = 0; i < num_particles; ++i){
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = heading(gen);
        p.weight = 1.;
        particles.push_back(p);
        
        weights.push_back(1.0);
    }
    
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // used for sensor noise
    default_random_engine gen;
    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
    std_x = std_pos[0];
    std_y = std_pos[1];
    std_theta = std_pos[2];

    normal_distribution<double> dist_x(0, std_x);
    normal_distribution<double> dist_y(0, std_y);
    normal_distribution<double> heading(0, std_theta);
    
    for(int i = 0; i < num_particles; ++i){
        Particle p = particles[i];
        if(fabs(yaw_rate) < 0.00001) {
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
        }
        else{
            double velocity_by_yaw_rate = velocity / yaw_rate;
            double x, y, theta;
            x = p.x + velocity_by_yaw_rate  * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            y = p.y + velocity_by_yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            theta = p.theta + yaw_rate * delta_t;
            
            // account for sensor noise by adding Gaussian noise
            p.x = x + dist_x(gen);
            p.y = y + dist_y(gen);
            p.theta = theta + heading(gen);
        }
        
        particles[i] = p;
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    if (predicted.size() == 0) {
        return;
    }
    
    for (int i = 0; i < observations.size(); ++i) {
        LandmarkObs obs = observations[i];
        LandmarkObs pred = predicted[0];
        int mapId = pred.id;
        float min_dist = sqrt((pred.x - obs.x) * (pred.x - obs.x) + (pred.y - obs.y) * (pred.y - obs.y));
        if (predicted.size() < 2) {
            observations[i].id = pred.id;
            continue;
        }
        for (int j = 1; j < predicted.size(); ++j) {
            pred = predicted[j];
            float dist = sqrt((pred.x - obs.x) * (pred.x - obs.x) + (pred.y - obs.y) * (pred.y - obs.y));
            if (dist < min_dist) {
                min_dist = dist;
                mapId = pred.id;
            }
        }
        
        // associate the observation with map landmark, using its id
        obs.id = mapId;
        observations[i] = obs;
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
    float sig_x = std_landmark[0];
    float sig_y = std_landmark[1];
    
    // 1) Transformation and association: predict measurements to all the map landmarks
    // within sensor range for each particle
    float mean = 0.;
    for (int i = 0; i < particles.size(); ++i) {
        // for each particle, transform the observation from the car's coordinate system
        // to the map's coordinate system
        Particle p = particles[i];
        std::vector<LandmarkObs> map_observations;
        for(int j = 0; j < observations.size(); ++j){
            LandmarkObs obs = observations[j];
            
            LandmarkObs map_obs;
            map_obs.id = obs.id;
            map_obs.x = p.x + (cos(p.theta) * obs.x) - (sin(p.theta) * obs.y);
            map_obs.y = p.y + (sin(p.theta) * obs.x) + (cos(p.theta) * obs.y);
            
            map_observations.push_back(map_obs);
        }
        
        // get the landmarks within the sensor range
        std::vector<LandmarkObs> predicted;
        for (int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
            Map::single_landmark_s landmark = map_landmarks.landmark_list[j];
            // within a rectangle region of size sensor_range?
            float side_x = fabs(landmark.x_f - p.x);
            float side_y = fabs(landmark.y_f - p.y);
            if (side_x <= sensor_range && side_y <= sensor_range) {
                LandmarkObs L;
                L.id = landmark.id_i;
                L.x = landmark.x_f;
                L.y = landmark.y_f;
                predicted.push_back(L);
            }
        }
        
        // associate each observation to the nearest landmark
        dataAssociation(predicted, map_observations);
        
        
        // 2) update the particle weights using the Multivariate-Gaussian Probability
        float w = 1.0; // reset the weight
        for(int j = 0; j < map_observations.size(); ++j){
            LandmarkObs obs = map_observations[j];
            // calculate normalization term
            float gauss_norm = (1 / (2 * M_PI * sig_x * sig_y));
            
            // calculate exponent
            // but first, get the landmark associated to the observation (nearest one)
            std::vector<LandmarkObs>::iterator it = std::find_if(predicted.begin(), predicted.end(), [&obs](const LandmarkObs& assoc_obj){
                return obs.id == assoc_obj.id;
            });
            float mu_x = it->x;
            float mu_y = it->y;
            float exponent = ((obs.x - mu_x) * (obs.x - mu_x)) / (2 * sig_x * sig_x) + ((obs.y - mu_y) * (obs.y - mu_y)) / (2 * sig_y * sig_y);
            float weight = gauss_norm * exp(-exponent);
            
             w *= weight;
        }
        
        p.weight = w;
        particles[i] = p;
        
        // normalization term
        mean += p.weight;
    }
    
    
    // normalize the weights
    for (int i = 0; i < particles.size(); ++i) {
        particles[i].weight /=  mean;
        weights[i] = particles[i].weight;
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    float max_weight = *max_element(weights.begin(), weights.end());
    
    default_random_engine gen;
    std::discrete_distribution<int> index_dist(0, num_particles - 1);
    int index = index_dist(gen);
    
    float beta = 0.0;
//    float mw2 = 2 * max_weight;
    std::uniform_real_distribution<double> weight_dist(0, max_weight);
    
    std::vector<Particle> resampled_particles;
    for (int i = 0; i < num_particles; ++i) {
        beta += weight_dist(gen) * 2;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }
    
    particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
