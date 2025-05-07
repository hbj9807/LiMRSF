# LiMRSF

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()  
[![License](https://img.shields.io/badge/license-MIT-blue)]()  

## Table of Contents

1. [Description](#description)  
2. [Features](#features)  
3. [Prerequisites & Installation](#prerequisites)  
4. [Contributing](#contributing)  
5. [License](#license)  
6. [Contact](#contact)

## Description
LiMRSF (LiDAR-MR-RGB Sensor Fusion) system overlaps 3D point cloud or 3D mesh on top of Mixed Reality Headset in real-time to detect blind spot in SLAM process and feedback intuitively to the user.

Main objective is to
- Point cloud registration error visualization
- ROS-TCP Server for data transfer
- Reduce time for site revisit

## Our Paper
https://arxiv.org/abs/2411.12514

## Our Videos
https://youtu.be/lripMy5RFcs?si=oT9suaIcNXGKk_XV

## Features
Unity scene for MR headset (HoloLens 2) to visualize transmitted mesh (.ply) with detected blind spots.

## Prerequisites & Installation
1. ROS
- Install ROS-TCP-Endpoint 
- git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git


2. Unity


2.1 Install Unity 2020.3.42f1


2.2 Install ROS-TCP-Connector


2.2.1 Using Unity 2020.2 or later, open the Package Manager from "Window" -> "Package Manager".


2.2.2 In the Package Manager window, find and click the "+" button in the upper lefthand corner of the window. Select "Add package from git URL...."


2.2.3 Enter https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector


## Contributing
This work was supported by the National Research Foundation of Korea (NRF) grant funded by the Korea government (MSIT) (Grant No. 2022M1A3C2085237). 

## License

## Contact
Hanbeom Chang - hanbeom.chang@stonybrook.edu
MEIC Lab. - https://www.meic-lab.com/
