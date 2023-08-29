# Aerial-robot-field-trial-original-data-and-programs-used
This repository contains the original data from the field trial and the programs used to get the data and operate the robot.
The gyro data folder contains the accelerometer and height sensor data, the soil moisture data consists of .txt files per replicate, and the lift data contains the linear potentiometer data during sensor insertion.
The aerial robot LabVIEW program contains the state machine logic that was used to control the sensor insertion, move the sensor, and record soil moisture data.
The Arduino program was used to measure soil moisture, send it to myRIO, and upload new flight plans so that the UAV could fly to the next location.
The R program was used to post-process the data and get tabulated forms of soil moisture data with the respective GPS coordinates.
