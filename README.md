# Noether

This package depends on PCL 1.8 and VTK 7.1.  Install VTK first and make sure that PCL is compiled against 7.1 or there will be run time errors.
## Prerequisites
These packages run on Ubuntu 16.04

### Installing VTK 7.1:
```
git clone https://github.com/Kitware/VTK.git
cmake .
make
sudo make install
```

### Installing PCL (compiled with VTK 7.1)
```
git clone https://github.com/PointCloudLibrary/pcl.git
```
open CMakeLists.txt
change line 364 from "find_package(VTK)" to  "find_package(VTK 7.1 REQUIRED)"
```
cmake .
make
sudo make install
```

## Build
TBD

## Test Noether
Run the unit tests for each package:
catkin_make run_tests_<package-name>
```
catkin_make run_tests_tool_path_planner
```

The noether package has an executable which is able to take in a mesh file (.stl format), read it, segment the file into adjacent surfaces,
and plan paths for each surface.  Work is in progress to read in point cloud (.pcd) files, but meshing results are not reliable right now.

