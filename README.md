# Localization with Particle Filters - Finding a Kidnapped Vehicle
Udacity SDCND Term 2, Project 3

## Project Basics
In this C++ project, I used a two-dimensional particle filter to help localize a car placed in an somewhat unknown location. I began by using less accurate map data (similar to GPS) to initialize my car's location, predicted my new location based on velocity and yaw (turn) rate, transformed sensor observation points into my map coordinates, associated these observations with landmarks on the map, and then calculated the likelihood that a given particle made those observation based off of the landmark positions in the map. The particles were then re-sampled based on how likely a given particle was to have made the observations of the landmarks, which helps to more accurately localize the vehicle.

Along with sufficient accuracy to have localized the vehicle within a small amount of space, the project also required having an efficient algorithm, as there was a time limit to how long it could run for without failing the given parameters.

### Project Steps
All steps within 'particle_filter.cpp' in the 'src' folder
* Initialization function (estimate position from GPS data using particle filters, add random noise)
* Prediction function (predict position based on adding velocity and yaw rate to particle filters, add random noise)
* Update Weights function - Transformation of observation points to map coordinates (given in vehicle coordinates)
* Update Weights function - Association of landmarks to the transformed observation points
* Update Weights function - Calculation of multi-variate Gaussian distribution
* Resample function - Resamples particles, with replacement occurring based on weighting distributions
* Optimizing algorithm - Performance within required accuracy and speed

## Running the Code
Once you have this repository on your machine, cd into the repository's root directory and run the following commands from the command line:
```
> ./clean.sh
> ./build.sh
> ./run.sh
```
This will then run in conjunction with the Term 2 simulator.

## Results
### Performance
The particle filter met the requirements, which were 1 meter in error for x and y translations, 0.05 rad in error for yaw, and 100 seconds of runtime for the particle filter. Please note that due to the random numbers generated in certain portions of my approach (for the Gaussian distributions), results may vary slightly. Error below is cumulative mean weighted error.

**Using: 100 particles**

Runtime: 17.219 seconds (based on old project version)

| Estim |  Error  |
| ----- | ------- |
|   x   | 0.113 | (m)
|   y   | 0.105 | (m)
|  yaw  | 0.004 | (rad)

### Visualizing the Car's Localization based on Landmark Observations
The green lines below are the car's observations of surrounding landmarks.
![Localizing](Localizing_screen.png)




