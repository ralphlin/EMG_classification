# EMG Classification.cpp

Written June 24, 2009
By Ralph Lin
University of Washington
Neurobotics Lab

## Description:
	This program is used to run the single ACT finger (6 DOF)
	by recognizing EMG signals from a human for the RSS 2009
	demo. It performs this by several steps:
		1. Reading in EMG data from EMG electrodes placed on three
		   muscles on the index finger using installed NiDAQ boards
		2. Classifying index finger motion as flex/extension,
		   adduction/abduction, clockwise circle, counterclockwise
		   circle, half curl, or full curl
		3. Output movement and period of motion to robot using UDP
		   protocol

## Notes:
	This program was designed to run with the corresponding 
	main_RSSdemo.cpp behavior on the ACTHand control machine as well
	as glsample.cpp to provide a on-screen avatar for finger motion

