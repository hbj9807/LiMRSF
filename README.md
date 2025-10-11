# LiMRSF (LiDAR-MR-RGB Sensor Fusion)

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()  
[![License](https://img.shields.io/badge/license-MIT-blue)]()  

## Abstract
**Authors**: `Hanbeom Chang` | `Jongseong Brad Choi` | `Chul Min Yeum`\
Indoor SLAM in confined spaces often yields drift, double walls, and coverage gaps that are discovered only after post-processing. We present LiMRSF, a handheld LiDAR-MR-RGB Sensor Fusion system that closes this loop by streaming an RGB-textured mesh with density-based blind spot overlays to a mixed reality headset, guiding targeted rescans in situ. Stage 1 acquires a colorized point cloud; Stage 2 performs Poisson Reconstruction meshing, blind spot highlighting, and QEM mesh simplification; Stage 3 delivers interative MR visualization and operator-driven rescanning with ICP alignment back into the map. Evaluation on three indoor sites shows that, at the operating point selected by F1 Score-max with a recall constraint, the blind spot detector achieves high area-weighted accuracy (mean Precision of 0.93, F1 Score of 0.67, IoU of 0.52). A geometry-space study confirms that simplification preserves inspection cues: bidirectional surface distances are 4-7 cm with highlight area recall 0.85-0.97 and modest boundary drift. We instrument the pipeline end-to-end: Stage 2 compute dominates refresh latency (330-865 s on our portable workstation), whereas network and client-apply are sub-second; across 30 trials per link/scene, one-way transfer delays average 146-1297 ms. Together, LiMRSF operationalizes a human-in-the-loop workflow that exposes uncertainty as a holographic cue, enabling on-site quality assurance and reducing revisits in BIM, inspection, and related indoor mapping tasks.

The paper can be found in [Arxiv](https://arxiv.org/abs/2411.12514).

A demo video can be found on our [YouTube](https://youtu.be/lripMy5RFcs?si=86RJgNUZksLTAuX1)

An explanatory video presentation can be found on our [YouTube](https://youtu.be/3EbnmCrZffk?si=n-qdLW7e8psmeuPf) 

## Table of Contents

1. [Description](#description)  
2. [Features](#features)  
3. [Prerequisites & Installation](#prerequisites)  
4. [Validation With Our Data](#validation-with-our-data)
5. [Contributing](#contributing)  
6. [License](#license)  
7. [Contact](#contact)

## 1. Description
Our `LiMRSF` (LiDAR-MR-RGB Sensor Fusion) system overlaps a 3D point cloud or 3D mesh on top of a Mixed Reality Headset (HoloLens 2) in real-time to detect blind spots in the SLAM process and feedback intuitively to the user.

`Main objective` is to
- Point cloud registration error visualization
- ROS-TCP Server for data transfer
- Reduce time for site revisit with Human-in-the-Loop

## 2. Features
Unity scene for MR headset (HoloLens 2) to visualize transmitted mesh (`.ply`) with detected blind spots.
- **ROS**: Opens ROS-TCP server and transfers data (blind spot highlighted mesh)
- **Unity**: Deployed in the app for visualizing the highlighted mesh in holograms

## 3. Prerequisites & Installation

### **3.1. Hardware**

`MR Headset`: Microsoft HoloLens 2  
`LiDAR`: Ouster OS0-32  
`IMU`: Built-in LiDAR in Ouster LiDAR  
`Camera`: FLIR Blackfly S

### **3.2. Software** 

**OS #1**: `Ubuntu 20.04`, ROS 1 `Noetic`  
**Drivers**: [Ouster Driver, Official](https://github.com/ouster-lidar/ouster-ros), [FLIR Driver, ros-drivers](https://github.com/ros-drivers/flir_camera_driver/tree/noetic-devel)  
**Calibration**: [LiDAR-Camera Calibration, Koide3](https://github.com/koide3/direct_visual_lidar_calibration)  
**Color Mapping**: [R3LIVE, hku-mars](https://github.com/hku-mars/r3live)  

**OS #2**: `Windows 10`  
`Download #1`: **Visual Studio 2019**  
Select `.NET desktop development`, `Desktop development with C++`, `Universal Windows Platform development`, `Game development with Unity`.
![alt text](image.png)
![alt text](image-1.png)

`Download #2`: **Unity Hub & Unity Editor**  
[Unity Hub Download](https://unity.com/download)  
[Unity Editor Download](https://unity.com/releases/editor/archive)  
We have tested our system with `Unity Editor 2020.3.42f1`

### 3.2.1 **ROS: ROS-TCP-Endpoint**  
This tool is for connecting ROS and Unity through TCP network in `ROS`.  
`git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git`

### 3.2.2 **Unity: ROS-TCP-Connector**: 
This tool is for connecting ROS and Unity through TCP network in `Unity`.  
Follow Installation Guide [Link](https://github.com/Unity-Technologies/ROS-TCP-Connector)

### 3.2.3 Install MRTK 2 in the Unity Scene
[Microsoft MRKT2 Guide](https://learn.microsoft.com/en-us/windows/mixed-reality/mrtk-unity/mrtk2/packages/mrtk-packages?view=mrtkunity-2022-05)

### **3.3 Configuration**

### 3.3.1 Clone the entire repository  
`git clone https://github.com/hbj9807/LiMRSF.git`

### 3.3.2 ROS Settings
`ROS_files` folder goes to your ROS system, and run with ROS-TCP-Endpoint.  

### 3.3.3 ROS-Unity TCP Settings
After installing `Unity` in `Windows platform`, in `ROSConnectionPrefab`, set `ROS IP Address` and `ROS Port` to the settings of your own `ROS-TCP-Endpoint's` IP and Port.  

![alt text](image-2.png)

## Validation With Our Data
1. Download our `Google Drive Dataset`: [https://drive.google.com/drive/u/1/folders/1et2u-7XFjwj3Bzm7ewrv6NoXhvH_1SC1](https://drive.google.com/drive/folders/1C-asrQ4nQnr937TkHoksUBUz2xdc7tX8?usp=sharing)  
All the evaluated data (.csv) are in the `Evaluation` folder.

3. Run `LiMRSF.py` in your ROS machine to generate `Mesh with Highlighted Result` and `Visualization in HoloLens 2`.
Confirm that your HoloLens 2 and ROS-TCP-Endpoint are connected with the same IP address.

4. Run `blindspot_eval_test.py` in any machine (ROS, Windows) to evaluate the blind spot detection precision.

5. Run `similarity_eval.py` in any machine (ROS, Windows) to evaluate the similarity between `Highlighted Mesh` and `Simplified Mesh`.


## Contributing
This work was supported by the National Research Foundation of Korea (NRF) grant funded by the Korea government (MSIT) (Grant No. 2022M1A3C2085237). 

## License
This project is licensed under the MIT License – see the [LICENSE](./LICENSE.md) file for details.

## Contact
Hanbeom Chang - hanbeom.chang@stonybrook.edu, hbj9807@gmail.com  
MEIC Lab. - https://www.meic-lab.com/
