# Robotics course 2020

## Abstract

The course project is divided in two parts.

## First part
### Given data
One Bag file, with data coming from two cars (GPS data: LLA format), racing in Monza ENI circuit.

### Tasks
- Publish both Odom and TF data for both cars
- Write a service that returns the distance between the two cars
- Publish at a given rate the distance of the two cars, and a flag that points out the security level based on the distance
- Use dynamic reconfiguration to modify the threshold of the flag.

## Second part

### Given data
Two Bag files, containing:

 - 3D PointCloud data captured by a robot, in an unknown enviroment (Oyster 64 planes LIDAR)
 - Visual odometry data (Intel T265) 
 - IMU sensors data

### Tasks
-  Create a map of the enviroment using OpenSlam's Gmapping  (SLAM) using one visual odometry by one of the two bags
- Configure the move_base package using AMCL localization system and the obtained map from previous task
- Test of the navigation stack, checking the correctness of the map
- Improve visual odometry using IMU (both sensors) and Optitrack data

### Professors & Tutors

-   Matteo Matteucci, DEIB department, Politecnico di Milano
-   Simone Mentasti, DEIB department, Politecnico di Milano
