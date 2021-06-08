# Visual Perception

## Visual Odometry for Localization in Autonomous Driving
I estimated an AV's trajectory by images taken with a monocular camera set up on the vehicle. 

In particular, I did
- Extract features from photos
- Use the extracted features to find matches between the features
- Use the found matches to estimate the camera motion between subsequent photos
- Use the estimated camera motion to build the vehicle trajectory

See the [Jupyter Notebook](Visual%20Odometry/Visual%20Odometry%20for%20Localization%20in%20Autonomous%20Driving.ipynb) for more details.

- - - -
## Environment Perception for Autonomous Driving
Given the output from 
- semantic segmentation neural networks
- 2D object detectors

I did
- Implement drivable space estimation in 3D
- Implement lane estimation
- Filter errors in the output of 2D object detectors
- Use the filtered 2D object detection results to determine how far obstacles are from the AV

See the [Jupyter Notebook](Environment%20Perception/Environment%20Perception%20For%20Self-Driving%20Cars%20-%20Learner%20-%20v1.ipynb) for more details.
