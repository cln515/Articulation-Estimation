A Hand Motion-guided Articulation and Segmentation Estimation.
====

Articulation and segmentation estimation in RGB-D image (For Kinect v2) using hand motion.

[Paper](http://arxiv.url)

## Description
This repository offers the object articulation and segmentation estimation using human hand motion detected in RGB-D image sequence. 

## Demo



## Requirement
Tested Environment: Windows 10

Dependency
- C++
    - OpenPose (https://github.com/CMU-Perceptual-Computing-Lab/openpose)
    - Kinect SDK V2 (https://www.microsoft.com/en-us/download/details.aspx?id=44561)
    - Point Cloud Library (http://www.pointclouds.org/)
    - Ceres solver (http://ceres-solver.org/)
- Python
    - Mask-RCNN (https://github.com/matterport/Mask_RCNN)
    - Hand Detection using TensorFlow (https://github.com/victordibia/handtracking)

## Usage
- Capture (C++) 
    - ```ArticulationDetection.exe --output_folder <capture_output_folder> --mode 0```
    - push keys for capture background and manimulation scene (push 'b' once and 'c' twice)
        -  c : start and stop recording
        -  b : capture background
        - esc: quit program
- Human Masking (Python)
    -  ```$ python masking.py </path/to/Mask_RCNN/> <capture_output_folder> <first_frame(int)> <last_frame(int)>```
- Processing (C++)
    - ```ArticulationDetection.exe --model_folder </path/to/openpose/models> --input_folder <capture_output_folder> --output_folder <result_output_folder> --first_frame <first_frame(int)> --last_frame <last_frame(int)> --mode 1```


## Install
### Dependency build
#### OpenPose
Clone repository
```
$ git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
```
CMake & Build
#### Kinect SDK V2
Download SDK V2 (https://www.microsoft.com/en-us/download/details.aspx?id=44561) and install

Copy "FindKinectSDK2.cmake" in (https://gist.github.com/UnaNancyOwen/b7f8a543c3fa91a1a407) to CMake modules folder (C:/Program Files/CMake/share/cmake-version/Modules/)

#### Point Cloud Library
Download and Install All-in-One package (http://unanancyowen.com/en/pcl181/) (VS 2015 or 2017)

#### Ceres Solver
Follow the installation instruction (http://ceres-solver.org/installation.html). 

CMake without GFLAGS and GLOG (check MINIGLOG and uncheck GFLAGS) to avoid to refer to duplicate libraries with OpenPose (OpenPose compiled in Windows refers to internal GFLAGS and GLOG).

Build "INSTALL" on Visual Studio. Install folder can be modified by configuring CMAKE_INSTALL_PREFIX.

### Build this source
Clone this repository
```
$git clone https://github.com/cln515/articulation-detection
```
Set source code folder and build folder

Run "configure"

Set OpenPose repository folder to OpenPose_DIR 

Set openpose.lib (/path/to/build_folder/src/openpose/Release/openpose.lib) to OpenPose_LIB 

Run "configure" again and "Generate"
## Licence

[MIT](https://github.com/LICENCE)

## Author

