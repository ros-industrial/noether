# Noether

Tool path planning and surface segmenter

---
## Prerequisites
- These packages run on Ubuntu 16.04 and ROS
- Uses the catkin tools build system

---
## Installation

This package depends on PCL 1.8 and VTK 7.1. 

#### Prerequisites
- **checkinstall**
    - `sudo apt install checkinstall`

#### Dependencies Installation
##### 1. VTK
1. Download [VTK 7.1](https://github.com/Kitware/VTK/releases/tag/v7.1.0)
2. Unzip or extract into a user accessible directory
3. `CD` into that directory and create a new `build` directory
4. Run cmake
    ```
    cd build
    cmake ..
    ```
1. Build the library
    ```
    make
    ```
    _This will take a while ..._
2. Install 
    ```
    sudo checkinstall --pkgname=vtk-7.1
    ```
    The installation process will prompt you to accept/reject some options prior to building the debian, **just follow the recommended prompts**.
    
    NOTE: Using `checkinstall` instead of `make install` has the advantage that it builds a debian package which can be easily uninstalled with `sudo dpkg -r [packagename]`.
    
##### 2. PCL 
1. Download [PCL 1.8](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.8.0)
2. Unzip or extract into a user accessible directory
3. `cd` into that directory and locate the `CMakeLists.txt` file.
4. Locate the `find package(VTK)` line (close to line 362) and edit it to `find_package(VTK 7.1 REQUIRED)`
5. Configure and build
    ```
    cmake .
    make
    ```
    _Wait until the build finishes, it may take a couple of hours ..._
1. Install 
    ```
    sudo checkinstall --pkgname=pcl-1.8
    ```
    The installation process will prompt you to accept/reject some options prior to building the debian, **just follow the recommended prompts**
    
    NOTE: Using `checkinstall` instead of `make install` has the advantage that it builds a debian package which can be easily uninstalled with `sudo dpkg -r [packagename]`.

---
## Build
- Create a catkin workspace and clone this repository in the `src` directory
- Build the noether packages
  ```
  catkin build noether noether_conversions
  ```
  
  _Note: You can just run catkin build in order to build everything in your workspace including the noether packages_

## Test Noether (DEPRECATED)
Run the unit tests for each package:
catkin_make run_tests_<package-name>
```
catkin_make run_tests_tool_path_planner
```

The noether package has an executable which is able to take in a mesh file (.stl format), read it, segment the file into adjacent surfaces,
and plan paths for each surface.  Work is in progress to read in point cloud (.pcd) files, but meshing results are not reliable right now.

