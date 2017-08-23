# 3D Mapping Using LiDAR

## Introduction
This document provides an overview of our process for developing point clouds using LiDAR, GPS, and an inertial measurement unit.

## Objectives / Requirements
* Gather live LiDAR data and store it in formatted text files.
* Generate 3D point cloud files that are geo-referenced using the LiDAR data file and a separately provided IMU data file.
* Generate point clouds files to be input into LASTools to be visualized.


## Hardware
* Vehicle
  *  DJI Spreading Wings S1000+
* Sensors
  * Velodyne PUCK VLP-16 (LIDAR)
  * MTi-G-710 IMU with GPS Antenna
* Data Processing
  * Intel NUC D54250WYK
  
 
## Basic Data Capture Procedure
 In the Debug folder, edit the Settings.txt file and put the number of seconds you want the code to wait before beginning to execute. This is for IMU calibration that can be read about in the User Manual found [here](https://www.xsens.com/products/mti-g-710/).
 
 Make sure lidarLiveCap.exe, Settings.txt and IMU.exe are all in the same folder before starting the data capture. lidarLiveCap refers to Settings and starts IMU so the data capture starts at approximately the same time.
  

