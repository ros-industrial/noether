# Noether

Tool path planning and surface segmenter

[![support level: consortium](https://img.shields.io/badge/support%20level-consortium-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

## Build Status

Platform             | CI Status
---------------------|:---------
Linux (Ubuntu)       | [![Build Status](https://github.com/ros-industrial/noether/workflows/Ubuntu/badge.svg)](https://github.com/ros-industrial/noether/actions)
Lint  (Clang-Format) | [![Build Status](https://github.com/ros-industrial/noether/workflows/Clang-Format/badge.svg)](https://github.com/ros-industrial/ros-industrial/actions)

## Supported OS/ROS Distributions
- Noetic (Ubuntu 20.04)

## Build

```
# if you already have a workspace, skip this step
mkdir -p ~/catkin_ws/src

# if you already cloned noether, skip this step
cd ~/catkin_ws/src && git clone https://github.com/ros-industrial/noether.git

# navigate to the root of your workspace
cd ~/catkin_ws

# pull down the dependencies
vcstool import src < src/noether/dependencies.rosinstall

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

## Run Applications and Demos
### Surface Raster Planner
The noether package has a *surface raster planner* executable which is able to take in a mesh file (.stl format), and generate raster paths on it, you can run it through the launch file with preconfigured parameters as follows:
```
roslaunch noether_examples plane_slicer_rastering_generator_demo.launch
```

or with your own mesh file
```
roslaunch noether_examples plane_slicer_rastering_generator_demo.launch mesh_file:=</absolute/path/to/my/mesh.stl>
```


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


