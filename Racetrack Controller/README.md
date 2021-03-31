# Racetrack Controller
I developed a controller to navigate a self-driving car around a racetrack in the CARLA simulation environment. 

I constructed logitudinal and lateral dynamic models for a vehicle and create controllers that regulate speed and path tracking performance using Python. 

In particular, for the implementation of the longitudinal controller, I used PID control as the high level controller to compute the acceleration, and then simply bypassing the lower level controller by using the acceleration as the throttle or brake output. 

