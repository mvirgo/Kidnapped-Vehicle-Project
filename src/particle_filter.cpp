/*
 * particle_filter.cpp
 *
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <map>

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
  random_device rd;
  default_random_engine gen(rd());
    
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
    
    if (abs(yaw_rate) != 0) {
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

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		vector<LandmarkObs> observations, Map map_landmarks) {
	// Updates the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
  // First, when iterating through each particle, need to transform observation points to map coordinates.
  // Next, associate each observation to its nearest landmark. The distribution can then be calculated.
  
  // First term of multi-variate normal Gaussian distribution calculated below
  // It stays the same so can be outside the loop
  const double a = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
  
  // The denominators of the mvGd also stay the same
  const double x_denom = 2 * std_landmark[0] * std_landmark[0];
  const double y_denom = 2 * std_landmark[1] * std_landmark[1];

  // Iterate through each particle
  for (int i = 0; i < num_particles; ++i) {
    
    // For calculating multi-variate Gaussian distribution of each observation, for each particle
    double mvGd = 1.0;
    
    // For each observation
    for (int j = 0; j < observations.size(); ++j) {
      
      // Transform the observation point (from vehicle coordinates to map coordinates)
      double trans_obs_x, trans_obs_y;
      trans_obs_x = observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta) + particles[i].x;
      trans_obs_y = observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta) + particles[i].y;
      
      // Find nearest landmark
      vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
      vector<double> landmark_obs_dist (landmarks.size());
      for (int k = 0; k < landmarks.size(); ++k) {
        
        // Down-size possible amount of landmarks to look at by only looking at those in sensor range of the particle
        // If in range, put in the distance vector for calculating nearest neighbor
        double landmark_part_dist = sqrt(pow(particles[i].x - landmarks[k].x_f, 2) + pow(particles[i].y - landmarks[k].y_f, 2));
        if (landmark_part_dist <= sensor_range) {
          landmark_obs_dist[k] = sqrt(pow(trans_obs_x - landmarks[k].x_f, 2) + pow(trans_obs_y - landmarks[k].y_f, 2));

        } else {
          // Need to fill those outside of distance with huge number, or they'll be a zero (and think they are closest)
          landmark_obs_dist[k] = 999999.0;
          
        }
        
      }
      
      // Associate the observation point with its nearest landmark neighbor
      int min_pos = distance(landmark_obs_dist.begin(),min_element(landmark_obs_dist.begin(),landmark_obs_dist.end()));
      float nn_x = landmarks[min_pos].x_f;
      float nn_y = landmarks[min_pos].y_f;
      
      // Calculate multi-variate Gaussian distribution
      double x_diff = trans_obs_x - nn_x;
      double y_diff = trans_obs_y - nn_y;
      double b = ((x_diff * x_diff) / x_denom) + ((y_diff * y_diff) / y_denom);
      mvGd *= a * exp(-b);
      
    }
    
    // Update particle weights with combined multi-variate Gaussian distribution
    particles[i].weight = mvGd;
    weights[i] = particles[i].weight;

  }
  
}

void ParticleFilter::resample() {
	// Resamples particles with replacement with probability proportional to their weight.
  
  // Vector for new particles
  vector<Particle> new_particles (num_particles);
  
  // Use discrete distribution to return particles by weight
  random_device rd;
  default_random_engine gen(rd());
  for (int i = 0; i < num_particles; ++i) {
    discrete_distribution<int> index(weights.begin(), weights.end());
    new_particles[i] = particles[index(gen)];
    
  }
  
  // Replace old particles with the resampled particles
  particles = new_particles;

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
