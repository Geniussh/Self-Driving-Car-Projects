# Kalman Filter
## EKF
There's a jupyter notebook in [EKF](EKF) implementing an Extended Kalman Filter that recursively extimated the position of a vehicle using a very simple type of LIDAR sensor, which returns range and bearing measurements corresponding to individual landmarks in the environment. I computed and implemented the Jacobians for the measurement models. The mechanism of EKF has been explained in the markdowns. I tuned the range/bearing measurement variances so that the vehicle follows perfectly along the ground truth trajectory.

## ES-EKF
I implemented the Error-State Extended Kalman Filter to localize a vehicle using data from the CARLA simulator. Explanation of ES-EKF can be found [here](ES-EKF/EKF%20FOR%20VEHICULAR%20STATE.pdf).

Specifically, I accomplished the following:
- Implemented the filter prediction step using data from IMU sensor
- Implemented the correction step using data from GPS and LIDAR sensors
- Examined the effects of sensor miscalibration by using an incorrect transform from the LIDAR frame to the IMU frame
- Implemented a systematic hyperparameter tuning script to tune the variance parameters for the estimator
- Examined the effects of sensor dropout where all external positioning information from GPS & LIDAR is lost for some time

### Sample outputs with tuned parameters for correct sensor calibration

<img src="https://github.com/Geniussh/Self-Driving-Car-Projects/blob/main/Demo%20Images/ES-EKF1.png" width="400px"><img src="https://github.com/Geniussh/Self-Driving-Car-Projects/blob/main/Demo%20Images/ES-EKF2.png" width="365px">

### Sample outputs under sensor dropout
<img src="https://github.com/Geniussh/Self-Driving-Cars-Projects/blob/main/Demo%20Images/ES-EKF3.png" height="300px"><img src="https://github.com/Geniussh/Self-Driving-Car-Projects/blob/main/Demo%20Images/ES-EKF4.png" height="300px">
