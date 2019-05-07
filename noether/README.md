# Noether

A ROS interface to the path planning library.

To test a `pcd` or `stl` file:
```
roslaunch noether surf_raster_planner_application.launch filename:=PATH_TO_STL_OR_PCD [tool:=PATH_TO_TOOL_YAML_FILE]
```

The `tool` parameter is optional. By default it looks to the `config/tool.yaml` file.
