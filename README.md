# hardware-accelerated-slam
Implementation of SLAM for PYNQ platform


## March 2nd update

The current state of my project is that the slam algorithm is implmented in Matlab along with functions to plot and anlyze the result of the algoithm to determine if it ran correctly. The implmentation of the algorithm istelf is done in `E3.m`. The file `ex4_slam.m` contains the top level function which calls the slam algorhtm as well as the functions to plot the output. The graphing functions are implemented in `visualize.m`. The graphing function also calculates the average error between the calculated robot and landmark positions comapred to the ground truth. 

The inputs to the algoithm are loaded in with 2 `.mat` files. These inputs are in the form of ground truth for the landmarks and robot for T timesteps. The inputs also contain parameters about the simulated sensor that the robot "uses" for the slam calculations, this includes a max range and sensor noise. The input also includes a sensor noise for the measured robot movement per time step. The slam algorithm returns the calculations for the location and confidence for each landmark the robot saw in addition to the location and confidence of the robot for each time step.

![](matlab_sample_output.jpg)

In the sample output above, the black stars represent the landmarks true location with the calculated locations represented as red dots with the ellipse representing thier confidence. Similarly, the blue line represents the ground truth of the robot with red ellipses representing the confidence for the each timestep.

### Running the Matlab implementation

To run the Matlab implmentation, you must first clone the repository, and then open Matlab to the folder `/hardware-accelerated-slam/`. In the Matlab Command Window, run the top-level function using the following command: `ex4_slam()`. This should produce an output similar to the image above.  

### Python implementation

There was a more than expected amount of effor that went into decoupling this implementation, mostly relating to the graphing functionality, from a Matlab specific toolbox. This is a nessecary step in moving the functionality to Python but it means that the Python implmentation is not functional yet. Most of the operations for matricies in matlab can be replaced by the NumPy library so many of the functions can be directly replaced. There is still more work to be done with smaller implmenation details such as starting indicies and graphing the output using a python plotting library. This work has been started in the similarly named python files to thier matlab equivilent and are located in `/slam_python/`