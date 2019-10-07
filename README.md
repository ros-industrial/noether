# Noether

Tool path planning and surface segmenter

---
## Prerequisites
- These packages run on Ubuntu 16.04 and ROS
- Uses the catkin tools build system

---
## Installation

This package depends on PCL 1.8 and VTK 8.2. 

#### Prerequisites
- **checkinstall**
    - `sudo apt install checkinstall`

#### Dependencies Installation
##### 1. VTK
1. Download [VTK 8.2](https://github.com/Kitware/VTK/releases/tag/v8.2.1)
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
    sudo checkinstall --pkgname=vtk-8.2
    ```
    The installation process will prompt you to accept/reject some options prior to building the debian, **just follow the recommended prompts**.
    
    NOTE: Using `checkinstall` instead of `make install` has the advantage that it builds a debian package which can be easily uninstalled with `sudo dpkg -r [packagename]`.
    
##### 2. PCL 
1. Download [PCL 1.8](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.8.1)
2. Unzip or extract into a user accessible directory
3. `cd` into that directory and locate the `CMakeLists.txt` file.
4. Locate the `find package(VTK)` line (close to line 362) and edit it to `find_package(VTK 8.2 REQUIRED)`
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
  catkin build noether
  ```
  
  _Note: You can just run catkin build in order to build everything in your workspace including the noether packages_

## Run Unit Tests
- Run the unit tests for a package:
    ```
    catkin run_tests tool_path_planner --no-deps
    ```
    
- Run all unit test in the noether repository
    ```
    catkin run_tests noether --no-deps
    ```
    >> NOTE: Press 'q' to close the vtk window and proceed with the test program.

## Run applications

The noether package has a *surface raster planner* executable which is able to take in a mesh file (.stl format), and generate raster paths on it, you can run it through the launch file with preconfigured parameters as follows:
```
roslaunch noether surf_raster_planner_application.launch filename:=</absolute/path/to/my/mesh.stl>
```

or you can run it with your own custom parameters as well by calling the node directly:
```
rosrun noether surface_raster_planner_application _pt_spacing:=0.05 _line_spacing:=0.15 _intersecting_plane_height:=0.05 _min_hole_size:=0.01 _min_segment_size:=0.01 _debug_on:=False _console_debug_on:=False _file_path:=</path/to/mesh/file.stl> 
```
The `debug_on` and `console_debug_on` argurments enable visual and console debugging respectively.  During visual debugging press 'q' on the vtk window in order to step through.

Work is in progress to read in point cloud (.pcd) files, but meshing results are not reliable right now.

