# Noether

This package depends on PCL 1.8.1 and VTK 7.1.1.  Install VTK first and make sure that PCL is compiled against 7.1 or there will be run time errors.

[![Build Status](https://travis-ci.org/ros-industrial/noether.svg?branch=master)](https://travis-ci.org/ros-industrial/noether)
## Prerequisites
These packages run on Ubuntu 16.04

### Installing VTK version v7.1.1:
```
git clone https://github.com/Kitware/VTK.git
cd VTK
git checkout tags/v7.1.1
mkdir build
cd build
cmake ..
make
sudo make install
```

### Installing PCL version pcl-1.8.1:
```
git clone https://github.com/PointCloudLibrary/pcl.git
```
PCL is compiled against VTK 7.1.1 so we need to make that explicit with the following two steps:
1. open CMakeLists.txt
2. change line 364 from "find_package(VTK)" to  "find_package(VTK 7.1.1 REQUIRED)"
Now, we can continue the build process:
```
cd pcl
mkdir build
cd build
cmake ..
make
sudo make install
```

## Test Noether
### catkin_make
Run the unit tests for each package:
catkin_make run_tests_<package-name>
```
catkin_make run_tests_tool_path_planner
```
### catkin tools
```
catkin run_tests noether
```

The noether package has an executable which is able to take in a mesh file (.stl format), read it, segment the file into adjacent surfaces,
and plan paths for each surface.  Work is in progress to read in point cloud (.pcd) files, but meshing results are not reliable right now.

