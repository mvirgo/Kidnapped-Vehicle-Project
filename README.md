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

> **NOTE**
> If you get any `command not found` problems, you will have to install 
> the associated dependencies (for example, 
> [cmake](https://cmake.org/install/))

If everything worked you should see something like the following output:

```
Time step: 2444
Cumulative mean weighted error: x .1 y .1 yaw .02
Runtime (sec): 38.187226
Success! Your particle filter passed!
```
## Results
### Performance
The particle filter met the requirements, which were 1 meter in error for x and y translations, 0.05 rad in error for yaw, and 45 seconds of runtime for the particle filter. Please note that due to the random numbers generated in certain portions of my approach (for the Gaussian distributions), results may vary slightly. Error below is cumulative mean weighted error.

**Using: 100 particles**

Runtime: 17.219 seconds

| Estim |  Error  |
| ----- | ------- |
|   x   | 0.11012 | (m)
|   y   | 0.10526 | (m)
|  yaw  | 0.00374 | (rad)

### Visualizing the Car's Localization based on Landmark Observations
The green lines below are the car's observations of surrounding landmarks. Please see separate visualizer branch in this repository for code necessary to reproduce these results in Udacity's simulator.
![Localizing](Localizing_screen.png)




