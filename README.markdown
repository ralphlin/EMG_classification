# EMG Classification.cpp

Written June 24, 2009  
By Ralph Lin  
University of Washington  
Neurobotics Lab  
Demo for RSS2009  

## Description:

This program is used to run the anatomically correct
testbed (ACT) finger, a 6 degree-of-freedom robotic finger
custom-designed to mimic human biomechanics. It performs 
this in 4 steps:

1. Reads in electrical muscle (EMG) impulses from electrodes placed on three muscles on the index finger using installed NiDAQ boards
2. Applies a Butterworth filter to smooth the raw data streams and segments into time windows.
3. Classifies index finger motion as flex/extension,
adduction/abduction, clockwise circle, counterclockwise
circle, half curl, or full curl based on parameters determined with an off-line supervised machine learning algorithm.
4. Commands robot to move in identified motion, matching speed and location of subject's finger using UDP communication protocol


## Notes:
This program was designed to run with the corresponding 
main_RSSdemo.cpp behavior on the ACTHand control machine as well
as glsample.cpp to provide a on-screen avatar for finger motion

