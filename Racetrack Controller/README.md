# Racetrack Controller

<p float="left">
  <img src="https://github.com/Geniussh/Self-Driving-Cars-Projects/blob/main/Racetrack%20Controller/Trajectory_Evaluation.png" width="400px">
  <img src="https://github.com/Geniussh/Self-Driving-Cars-Projects/blob/main/Racetrack%20Controller/Velocity_Evaluation.png" width="400px">
</p>

I developed a controller to navigate a self-driving car around a racetrack in the CARLA simulation environment. 

I constructed logitudinal and lateral dynamic models for a vehicle and create controllers that regulate speed and path tracking performance using Python. 

In particular, for the implementation of the longitudinal controller, I used PID control as the high level controller to compute the acceleration, and then simply bypassing the lower level controller by using the acceleration as the throttle or brake output. 

I used [Stanley Controller](https://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf) as the implementation of the lateral controller. 

I tested the limits of my control design and the testing results are shown as follows (with respect to the reference path and reference velocity along the waypoints):


### To Run the Controller in CARLA
- Place the current directory (as a directory) under ``/path_to_CARLA_root/PythonClient``.
- ``
./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30
`` in Ubuntu  
``
CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30
`` in Windows  
- ``
python ./PythonClient/Racetrack\ Controller/module.py
``
