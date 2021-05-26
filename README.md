# Self-Driving-Cars-Projects
I completed multiple autonomous vehicle (AV) projects using the open source simulator [CARLA](https://github.com/carla-simulator/carla/). These projects span the domains of AV from control, state estimation, localization, perception to motion planning. 

## [Racetrack Controller](Racetrack%20Controller)
Controller to navigate a self-driving car around a racetrack, using CARLA simulation environment.

<img src="https://github.com/Geniussh/Self-Driving-Cars-Projects/blob/main/Demo%20Images/racetrack.png" width="500px">

## [ES-EKF Estimator](ES-EKF%20Estimator)
Error-State Extended Kalman Filter to localize a vehicle using data from the CARLA simulator. Also including implementation for a vanilla Extended Kalman Filter.

<img src="https://github.com/Geniussh/Self-Driving-Cars-Projects/blob/main/Demo%20Images/ESEKF.png" width="500px">

## [Visual Perception](Visual%20Perception)
Algorithms to build vehicle trajectory and algorithms for environment perception stack in AVs. Also including implementation for a visual odometry.

<p float="left">
  <img src="https://github.com/Geniussh/Self-Driving-Cars-Projects/blob/main/Demo%20Images/VP1.png" width="300px">
  <img src="https://github.com/Geniussh/Self-Driving-Cars-Projects/blob/main/Demo%20Images/VP2.png" width="300px">
</p>

## [Motion Planning](Motion%20Planning)
A functional motion planning stack that can avoid both static and dynamic obstacles while tracking the center line of a lane as well as handling stop signs, in CARLA. Also including implementations for a mission planner and occupancy grid in Jupyter Notebooks. 

<img src="https://github.com/Geniussh/Self-Driving-Cars-Projects/blob/main/Demo%20Images/MotionPlanner.png" width="500px">

# Install CARLA
### Ubuntu
Download [CarlaUE4 for Ubuntu](https://drive.google.com/file/d/1F35snQj1NTo4u0EaUFL2nE8C67Vg8UCs/view?usp=sharing) and follow the [instructions](Demo%20Images/CARLA-Setup-Guide-_Ubuntu.pdf).

### Windows
Download [CarlaUE4 for Windows](https://drive.google.com/file/d/1EH3aXkSiwt0AqImD4kwnAYGb4b8hrGFn/view?usp=sharing) and follow the [instructions](Demo%20Images/CARLA-Setup-Guide-_Windows.pdf).
