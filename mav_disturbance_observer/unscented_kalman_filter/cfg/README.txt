The dynamically configurable parameters are the diagonal entries of the process noise (Q) and measurement noise (R) covariance matrices for the UKF. 
use dynamic reconfigure by calling in the terminal  
$ rosrun rqt_reconfigure rqt_reconfigure

To create a new dynamic reconfigure package, create a new file.cgf, load its location in the terminal and exectute the command:
$ chmod a+x cfg/file.cfg
