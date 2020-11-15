# Noether

Tool path planning and surface segmenter

[![support level: consortium](https://img.shields.io/badge/support%20level-consortium-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

## Build Status

Platform             | CI Status
---------------------|:---------
Linux (Focal)        | [![Build Status](https://github.com/ros-industrial/noether/workflows/Focal-Build/badge.svg)](https://github.com/ros-industrial/noether/actions)
Linux (Bionic)       | [![Build Status](https://github.com/ros-industrial/noether/workflows/Bionic-Build/badge.svg)](https://github.com/ros-industrial/noether/actions)
Linux (Xenial)       | [![Build Status](https://github.com/ros-industrial/noether/workflows/Xenial-Build/badge.svg)](https://github.com/ros-industrial/noether/actions)
Lint  (Clang-Format) | [![Build Status](https://github.com/ros-industrial/noether/workflows/Clang-Format/badge.svg)](https://github.com/ros-industrial/ros-industrial/actions)

---
## Prerequisites
- These packages run on Ubuntu 16.04 and ROS
- Uses the catkin tools build system

---
## Installation

This package depends on PCL 1.9.1+ and VTK 7.1+. If you are using a Ubuntu system version 20.04+ (ROS Noetic/Foxy), you should already have these and can skip to the build instructions. Otherwise proceed with installing the custom PCL and VTK versions.

#### Prerequisites
- **checkinstall**
    - `sudo apt install checkinstall`

#### Useful Information
- **number of available threads / CPU cores**
    - This is useful for accelerating building of the required libraries, but is not required.  If you know this number, great!  If not, you can use one of these commands:
    - `htop` - You can count the number of threads displayed at the top.
    - `lscpu` - Outputs a number of CPUs, or you can multiply `Thread(s) per core:` * `Core(s) per socket:` * `Socket(s)` to get the available threads

#### Dependencies Installation
##### 1. VTK
1. Download [VTK 7.1](https://github.com/Kitware/VTK/archive/v7.1.1.tar.gz)
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

    If you know the number of threads available, you can specify to use more of them.  Leaving one or two open should allow you to continue using your computer.  For example, if you have 8 threads available, you could do the following to speed up build time:
    ```
    make --jobs=6
    ```
2. Install 
    ```
    sudo checkinstall --pkgname=vtk-7.1
    ```
    The installation process will prompt you to accept/reject some options prior to building the debian, **just follow the recommended prompts**.
    
    NOTE: Using `checkinstall` instead of `make install` has the advantage that it builds a debian package which can be easily uninstalled with `sudo dpkg -r [packagename]`.
    
##### 2. PCL 
1. Download [PCL 1.9.1](https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz)
2. Unzip or extract into a user accessible directory
3. `cd` into that directory and locate the `CMakeLists.txt` file.
4. Locate the `find package(VTK)` line (close to line 362) and edit it to `find_package(VTK 7.1 REQUIRED)`
5. Configure the build using the ccmake gui
    ```
    mkdir build
    cd build
    ccmake ..
    ```
    - Then locate the `BUILD_surface_on_nurbs` flag and set it to `ON`
    - Locate the `PCL_ENABLE_SSE` flag and set it to `OFF` or `FALSE`, this alters the alignment of class members when enabled, see [here](https://github.com/PointCloudLibrary/pcl/issues/1725)
6. Build
    ```
    make
    ```
    _Wait until the build finishes, it may take a couple of hours ..._

    If you know the number of threads available, you can specify to use more of them.  Leaving one or two open should allow you to continue using your computer.  For example, if you have 8 threads available, you could do the following to speed up build time:
    ```
    make --jobs=6
    ```
1. Install 
    ```
    sudo checkinstall --pkgname=pcl-1.9.1
    ```
    The installation process will prompt you to accept/reject some options prior to building the debian, **just follow the recommended prompts**
    
    NOTE: Using `checkinstall` instead of `make install` has the advantage that it builds a debian package which can be easily uninstalled with `sudo dpkg -r [packagename]`.

---
## Build

```
# if you already have a workspace, skip this step
mkdir -p ~/catkin_ws/src

# if you already cloned noether, skip this step
cd ~/catkin_ws/src && git clone https://github.com/ros-industrial/noether.git

# navigate to the root of your workspace
cd ~/catkin_ws

# pull down the dependencies
vcstool import src < src/noether/dependencies_ros1.rosinstall

# build the Noether packages
# note: if you want to build the entire workspace, just run 'catkin build' instead
catkin build noether
```

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

---

## Run Applications and Demos
### Surface Raster Planner
The noether package has a *surface raster planner* executable which is able to take in a mesh file (.stl format), and generate raster paths on it, you can run it through the launch file with preconfigured parameters as follows:
```
roslaunch noether_examples surf_raster_planner_demo.launch
```

or with your own mesh file
```
roslaunch noether_examples surf_raster_planner_demo.launch filename:=</absolute/path/to/my/mesh.stl>
```

Also, the node can be called directly with custom parameters directly:
```
rosrun noether surface_raster_planner_application _pt_spacing:=0.05 _line_spacing:=0.15 _intersecting_plane_height:=0.05 _min_hole_size:=0.01 _min_segment_size:=0.01 _debug_on:=False _console_debug_on:=False _file_path:=</absolute/path/to/my/mesh.stl>
```
The `debug_on` and `console_debug_on` argurments enable visual and console debugging respectively.  During visual debugging press 'q' on the vtk window in order to step through.

Work is in progress to read in point cloud (.pcd) files, but meshing results are not reliable right now.

### Mesh Filtering Demo
The mesh filtering demo applies a bspline smoothing algorithm to a noisy mesh, run the following to see it in action:
```
roslaunch noether_examples mesh_filtering_demo.launch
```

- Filter your own *ply* mesh file
```
roslaunch noether_examples mesh_filtering_demo.launch mesh_file:=/path/to/my/mesh.ply
```

See [here](noether_filtering/README.md) for more on the filter types available,  
Custom mesh filter plugins can be added by inheriting from the `noether_filtering::mesh::MeshBaseFilter` class.

### Generate Edge Paths Demo
Runs an algorithm that identifies all the half edges that constiture the boundary
- On a dummy *ply* mesh file
```
roslaunch noether_examples halfedge_finder_demo.launch
```

- On your own *ply* mesh file
```
roslaunch noether_examples halfedge_finder_demo.launch mesh_file:=</absolute/path/to/my/mesh.stl>
```


