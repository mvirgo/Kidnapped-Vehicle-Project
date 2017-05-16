/*
 * particle_filter.cpp
 *
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Sets the number of particles. Initializes all particles to first position (based on estimates of
  // x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Adds random Gaussian noise to each particle.
  
  if(!is_initialized) {
    // Set number of particles
    // *** Can be tuned ***
    num_particles = 100;
    
    // Size weights vector based on num_particles
    std::vector<double> weights (num_particles);
    
    // Initialize vector of particles
    std::vector<Particle> particles (num_particles);
    
    // Engine for later generation of particles
    std::default_random_engine gen;
    
    // Creates a normal (Gaussian) distribution for x, y and theta (yaw).
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);
    
    // Initializes particles - from the normal distributions set above
    for (int i = 0; i < num_particles; ++i) {
      double particle_x, particle_y, particle_theta;
      
      // Generate particle data
      particle_x = dist_x(gen);
      particle_y = dist_y(gen);
      particle_theta = dist_theta(gen);
      
      // Add generated particle data to particles class
      particles[i].id = i;
      particles[i].x = particle_x;
      particles[i].y = particle_y;
      particles[i].theta = particle_theta;
      particles[i].weight = 1;
      
      // Show as initialized; no need for prediction yet
      is_initialized = true;
      return;
      
    }
    
  }
  // *** Add in call to predict function if initialized ***
  // *** After prediction need to update weights, resample ***

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

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
