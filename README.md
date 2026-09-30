# Kalman Filter SLAM

This project is an implementation of a Kalman filter for 2D SLAM (Simultaneous Localization And Mapping) with a drone. \


A drone moves through an unknown 2D environment, with access only to its own (noisy) odometry and noisy measurements of nearby landmarks. The Kalman filter estimates, simultaneously, the drone's position and the landmarks' positions, even though neither is known beforehand.

![Filter result](partial_observation/images/partial_obs.png)

## Project structure

The project is organized in two parts, of increasing complexity:

- **[`full_observation/`](full_observation/)** — baseline case: all landmarks are observed at every step, always in the same order. This part establishes the core mechanics of the filter.
- **[`partial_observation/`](partial_observation/)** — a more realistic case: only a subset of landmarks is observed at each step, in an unknown order. This requires an additional data association step (determining which perceived point corresponds to which known landmark). This part also includes a simulator that generates test data with a known ground truth.

Each folder contains its own README with further details.

## Running the code

The implementation relies solely on base MATLAB (no additional toolbox), and was tested on MATLAB Online. Running a given part only requires opening `main.m` in the corresponding folder (the data file is already provided in the `data/` subfolder) and executing it.

---
*Project developed as part of the MO102 course (ENSTA Paris).*
