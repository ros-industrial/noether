# Noether Filtering

This package provides a generic manager for dynamically constucting and running filter pipelines.
At run-time, a user can specify the configuration of various filter groups (i.e. pipelines) to the manager using XmlRpc values.
A filter group is comprised of a serial chain of filters through which to run data. The manager loads the filters
from plugins and performs the filtering operations on an input data object.

The target applications for this pipeline are mesh and point cloud types, but could be
extended to any other classes where filter pipelines apply (such as 2D images).

## Usage

The filter manager is configured using XmlRpc value organized in the structure shown below:

```
- filter_groups:
  - group_name: (my_group)
    filters:
    - name: (filter_1)
      type: noether_filtering::(FilterClass)
      config:
      - param_1: (my_param_1)
        param_2: (my_param_2)
    - name: (filter_2)
      type: noether_filtering::(FilterClass)
      config:
      - param_a: (my_param_a)
        param_b: (my_param_b)
```

Required Configuration Parameters:
  - Filter manager configuration
    - `filter_groups`: array, filter group configurations
  - Filter group configuration:
    - `group_name`: string, unique name of the filter group
    - `continue_on_failure`: boolean, whether or not the chain should continue after failure of one filter
    - `filters`: array, filter configurations
  - Filter configuration
    - `name`: string, unique name of the filter
    - `type`: string, fully qualified filter class name (see Available Plugins below)
    - `config`: XmlRpc value, structure containing configuration data of the individual filter

## Supported Types

This package compiles various filters for the `pcl::PolygonMesh` type and for
[all PCL XYZ point types](https://github.com/PointCloudLibrary/pcl/blob/a8f6435a1a6635656327d5347fe81b1876a11dea/common/include/pcl/impl/point_types.hpp#L112)

## Available Plugins

The following plugins are currently available and are listed by filter data type

### `pcl::PolygonMesh`
- `noether_filtering::BSplineReconstruction`

### `pcl::PointCloud`
- `noether_filtering::CropBoxFilter<PointT>`
- `noether_filtering::PassThrough<PointT>`
- `noether_filtering::RadiusOutlierFilter<PointT>`
- `noether_filtering::StatisticalOutlierFilter<PointT>`
- `noether_filtering::VoxelGridFilter<PointT>`

***Note:*** *a valid `PointT` type must be provided in the specification of the name of the plugin
(i.e. `pcl::PointXYZ`)*
