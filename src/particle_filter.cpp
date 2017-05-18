/*
 * particle_filter.cpp
 *
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Sets the number of particles. Initializes all particles to first position (based on estimates of
  // x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Adds random Gaussian noise to each particle.
  
  // Set number of particles
  // *** Can be tuned ***
  num_particles = 100;
    
  // Resize weights vector based on num_particles
  weights.resize(num_particles);
    
  // Resize vector of particles
  particles.resize(num_particles);
  
  // Engine for later generation of particles
  default_random_engine gen;
    
  // Creates a normal (Gaussian) distribution for x, y and theta (yaw).
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
    
  // Initializes particles - from the normal distributions set above
  for (int i = 0; i < num_particles; ++i) {
      
    // Add generated particle data to particles class
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
      
  }
    
  // Show as initialized; no need for prediction yet
  is_initialized = true;
  return;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Adds measurements for velocity and yaw_rate to each particle and adds random Gaussian noise.
  
  // Engine for later generation of particles
  default_random_engine gen;
  
  // Make distributions for adding noise
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);
  
  // Different equations based on if yaw_rate is zero or not
  for (int i = 0; i < num_particles; ++i) {
    
    if (abs(yaw_rate) > 1e-6) {
      // Add measurements to particles
      particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
      particles[i].theta += yaw_rate * delta_t;
      
    } else {
      // Add measurements to particles
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
      // Theta will stay the same due to no yaw_rate
      
    }

    // Add noise to the particles
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
    
  }
  
  return;

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		vector<LandmarkObs> observations, Map map_landmarks) {
	// Updates the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  
  // Iterate through each particle
  for (int i = 0; i < num_particles; ++i) {
    
    // Vector to hold multi-variate Gaussian distribution of each observation
    vector<LandmarkObs> mvGd;
    
    // For each observation
    for (int j = 0; j < observations.size(); ++j) {
      
      // Transform the observation point (from vehicle coordinates to map coordinates)
      observations[j].x = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) + particles[i].x;
      observations[j].y = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) + particles[i].y;
      
      // Find nearest landmark
      vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
      vector<double> landmark_obs_dist (landmarks.size());
      for (int k = 0; k < landmarks.size(); ++k) {
        
        // Down-size possible amount of landmarks to look at by only looking at those in sensor range of the particle
        // If in range, put in the distance vector for calculating nearest neighbor
        double landmark_part_dist = sqrt(pow(particles[i].x - landmarks[k].x_f, 2) + pow(particles[i].y - landmarks[k].y_f, 2));
        if (landmark_part_dist <= sensor_range) {
          landmark_obs_dist[k] = sqrt(pow(observations[j].x - landmarks[k].x_f, 2) + pow(observations[j].y - landmarks[k].y_f, 2));
          
        }
        
      }
      
      // Associate the observation point with its nearest landmark neighbor
      int min_pos = distance(landmark_obs_dist.begin(),min_element(landmark_obs_dist.begin(),landmark_obs_dist.end()));
      float nn_x = landmarks[min_pos].x_f;
      float nn_y = landmarks[min_pos].y_f;
      
      // Calculate multi-variate Gaussian distribution
      //mvGd[j] = multi-variate Gaussian distribution equation
      
    }
    
    // Update weights with combined multi-variate Gaussian distribution
    //double comb_mvGd = 0.0;
    //for (int n : mvGd) comb_mvGd *= n;
    //particles[i].weight = comb_mvGd;
    
  }
  
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(string filename) {
	// You don't need to modify this file.
	ofstream dataFile;
	dataFile.open(filename, ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
