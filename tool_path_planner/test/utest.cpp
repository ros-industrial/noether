
/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <gtest/gtest.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkIdTypeArray.h>
#include <tool_path_planner/surface_walk_raster_generator.h>
#include <tool_path_planner/plane_slicer_raster_generator.h>
#include <tool_path_planner/eigen_value_edge_generator.h>
#include <tool_path_planner/halfedge_edge_generator.h>
#include <tool_path_planner/utilities.h>

#define DISPLAY_LINES 1
#define DISPLAY_NORMALS 0
#define DISPLAY_DERIVATIVES 1
#define POINT_SPACING 1.0

vtkSmartPointer<vtkPolyData> loadTempMesh()
{
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
  reader->SetFileName ("/tmp/test_mesh.ply");
  reader->Update ();
  polydata = reader->GetOutput ();
  printf("Loaded /tmp/test_mesh.ply with %ld points/vertices.\n", polydata->GetNumberOfPoints ());
  return polydata;
}

vtkSmartPointer<vtkPolyData> createTestMesh1(double sample_spacing = 0.5)
{
  // Get mesh
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane(30, 10, vtk_viewer::FLAT);
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(points, sample_spacing, 5);
  vtk_viewer::generateNormals(data);

  // create cutout in the middle of the mesh
  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  double pt1[3] = { 2.0, 3.0, 0.0 };
  double pt2[3] = { 4.0, 2.0, 0.0 };
  double pt3[3] = { 5.0, 3.0, 0.0 };
  double pt4[3] = { 4.0, 5.0, 0.0 };
  points2->InsertNextPoint(pt1);
  points2->InsertNextPoint(pt2);
  points2->InsertNextPoint(pt3);
  points2->InsertNextPoint(pt4);

  vtkSmartPointer<vtkPolyData> data2 = vtkSmartPointer<vtkPolyData>::New();
  data2 = vtk_viewer::cutMesh(data, points2, false);
  return data2;
}

vtkSmartPointer<vtkPolyData> createTestMesh2(double sample_spacing = 0.5)
{
  // Get mesh
  vtkSmartPointer<vtkPoints> points = vtk_viewer::createPlane();
  vtkSmartPointer<vtkPolyData> data = vtk_viewer::createMesh(points, sample_spacing, 5);
  vtk_viewer::generateNormals(data);

  // create cutout in the middle of the mesh
  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
  double pt1[3] = { 2.0, 3.0, 0.0 };
  double pt2[3] = { 4.0, 2.0, 0.0 };
  double pt3[3] = { 5.0, 3.0, 0.0 };
  double pt4[3] = { 4.0, 5.0, 0.0 };
  points2->InsertNextPoint(pt1);
  points2->InsertNextPoint(pt2);
  points2->InsertNextPoint(pt3);
  points2->InsertNextPoint(pt4);

  vtkSmartPointer<vtkPolyData> data2 = vtkSmartPointer<vtkPolyData>::New();
  data2 = vtk_viewer::cutMesh(data, points2, false);

  return data2;
}

void runRasterRotationTest(tool_path_planner::PathGenerator& planner, vtkSmartPointer<vtkPolyData> mesh)
{
  // Set input mesh
  planner.setInput(mesh);

  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  double scale = 1.0;

  // Display mesh results
  color[0] = 0.9f;
  color[1] = 0.9f;
  color[2] = 0.9f;
  viz.addPolyDataDisplay(mesh, color);

  // Display surface normals
  if (DISPLAY_NORMALS)
  {
    color[0] = 0.9f;
    color[1] = 0.1f;
    color[2] = 0.1f;
    vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
    normals_data = planner.getInput();
    viz.addPolyNormalsDisplay(normals_data, color, scale);
  }

  // Plan paths for given mesh
  boost::optional<tool_path_planner::ToolPaths> paths = planner.generate();
  ASSERT_TRUE(paths);
  tool_path_planner::ToolPathsData paths_data = tool_path_planner::toToolPathsData(paths.get());
  for (std::size_t i = 0; i < paths_data.size(); ++i)
  {
    for (std::size_t j = 0; j < paths_data[i].size(); ++j)
    {
      if (DISPLAY_LINES)  // display line
      {
        color[0] = 0.2f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths_data[i][j].line, color, scale);
      }

      if (DISPLAY_DERIVATIVES)  // display derivatives
      {
        color[0] = 0.9f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths_data[i][j].derivatives, color, scale);
      }
    }
  }

#ifdef NDEBUG
  // release build stuff goes here
  CONSOLE_BRIDGE_logError("noether/tool_path_planner test: visualization is only available in debug mode");
#else
  // Debug-specific code goes here
  viz.renderDisplay();
#endif
}

void runTestCase1(tool_path_planner::PathGenerator& planner, vtkSmartPointer<vtkPolyData> mesh)
{
  // Set input mesh
  planner.setInput(mesh);

  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  double scale = 1.0;

  // Display mesh results
  color[0] = 0.9f;
  color[1] = 0.9f;
  color[2] = 0.9f;
  viz.addPolyDataDisplay(mesh, color);

  // Display surface normals
  if (DISPLAY_NORMALS)
  {
    color[0] = 0.9f;
    color[1] = 0.1f;
    color[2] = 0.1f;
    vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
    normals_data = planner.getInput();
    viz.addPolyNormalsDisplay(normals_data, color, scale);
  }

  // Plan paths for given mesh
  boost::optional<tool_path_planner::ToolPaths> paths = planner.generate();
  ASSERT_TRUE(paths);
  tool_path_planner::ToolPathsData paths_data = tool_path_planner::toToolPathsData(paths.get());
  for (std::size_t i = 0; i < paths_data.size(); ++i)
  {
    for (std::size_t j = 0; j < paths_data[i].size(); ++j)
    {
      if (DISPLAY_LINES)  // display line
      {
        color[0] = 0.2f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths_data[i][j].line, color, scale);
      }

      if (DISPLAY_DERIVATIVES)  // display derivatives
      {
        color[0] = 0.9f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths_data[i][j].derivatives, color, scale);
      }
    }
  }

#ifdef NDEBUG
  // release build stuff goes here
  CONSOLE_BRIDGE_logError("noether/tool_path_planner test: visualization is only available in debug mode");
#else
  // Debug-specific code goes here
  viz.renderDisplay();
#endif
}

void runTestCaseRansac(tool_path_planner::PathGenerator& planner, vtkSmartPointer<vtkPolyData> mesh)
{
  // Set input mesh
  planner.setInput(mesh);

  vtk_viewer::VTKViewer viz;
  std::vector<float> color(3);

  double scale = 1.0;

  // Display mesh results
  color[0] = 0.9f;
  color[1] = 0.9f;
  color[2] = 0.9f;
  viz.addPolyDataDisplay(mesh, color);

  // Display surface normals
  if (DISPLAY_NORMALS)
  {
    color[0] = 0.9f;
    color[1] = 0.1f;
    color[2] = 0.1f;
    vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
    normals_data = planner.getInput();
    viz.addPolyNormalsDisplay(normals_data, color, scale);
  }

  // Plan paths for given mesh
  boost::optional<tool_path_planner::ToolPaths> paths = planner.generate();
  ASSERT_TRUE(paths);
  tool_path_planner::ToolPathsData paths_data = tool_path_planner::toToolPathsData(paths.get());
  for (std::size_t i = 0; i < paths_data.size(); ++i)
  {
    for (std::size_t j = 0; j < paths_data[i].size(); ++j)
    {
      if (DISPLAY_LINES)  // display line
      {
        color[0] = 0.2f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths_data[i][j].line, color, scale);
      }

      if (DISPLAY_DERIVATIVES)  // display derivatives
      {
        color[0] = 0.9f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths_data[i][j].derivatives, color, scale);
      }
    }
  }

#ifdef NDEBUG
  // release build stuff goes here
  CONSOLE_BRIDGE_logError("noether/tool_path_planner test: visualization is only available in debug mode");
#else
  // Debug-specific code goes here
  viz.renderDisplay();
#endif
}

void runExtraRasterTest(tool_path_planner::PathGenerator& planner,
                        tool_path_planner::PathGenerator& planner_with_extras,
                        vtkSmartPointer<vtkPolyData> mesh,
			double scale = 1.0)
{
  // Set input mesh
  planner.setInput(mesh);
  planner_with_extras.setInput(mesh);

  // Test on simple surface without extras
  boost::optional<tool_path_planner::ToolPaths> paths_no_extras = planner.generate();
  ASSERT_TRUE(paths_no_extras);

  // Test on simple surface with extras
  boost::optional<tool_path_planner::ToolPaths> paths_with_extras = planner_with_extras.generate();
  ASSERT_TRUE(paths_with_extras);

  // Check that the number of rasters has increased by 2
  ASSERT_EQ(paths_no_extras.get().size() + 2, paths_with_extras.get().size());

  for (tool_path_planner::ToolPath path : *paths_no_extras)
  {
    for (tool_path_planner::ToolPathSegment seg : path)
    {
      double average_point_spacing = 0.;
      int n_pts = 0;
      Eigen::Isometry3d prev_waypoint = seg[0];
      for (Eigen::Isometry3d waypoint : seg)
      {
        Eigen::Vector3d v = waypoint.translation() - prev_waypoint.translation();
        average_point_spacing += sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
        prev_waypoint = waypoint;
        n_pts++;
      }
      average_point_spacing = average_point_spacing / (n_pts - 1);
      //	  ASSERT_NEAR(POINT_SPACING, average_point_spacing, POINT_SPACING*.25);
    }
  }

  for (tool_path_planner::ToolPath path : *paths_with_extras)
  {
    for (tool_path_planner::ToolPathSegment seg : path)
    {
      double average_point_spacing = 0.;
      int n_pts = 0;
      Eigen::Isometry3d prev_waypoint = seg[0];
      for (Eigen::Isometry3d waypoint : seg)
      {
        Eigen::Vector3d v = waypoint.translation() - prev_waypoint.translation();
        double dist = sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
        printf("dist = %lf\n", dist);
        average_point_spacing += dist;
        prev_waypoint = waypoint;
        n_pts++;
      }
      average_point_spacing = average_point_spacing / (n_pts - 1);
      printf("average_point_spacing = %lf\n", average_point_spacing);
      //	  ASSERT_NEAR(POINT_SPACING, average_point_spacing, POINT_SPACING*.25);
    }
  }

#ifdef NDEBUG
  // release build stuff goes here
  CONSOLE_BRIDGE_logInform("noether/tool_path_planner test: visualization is only available in debug mode");

#else
  // Debug-specific code goes here
  vtk_viewer::VTKViewer viz;
  vtk_viewer::VTKViewer viz2;
  std::vector<float> color(3);

  // Display mesh results
  color[0] = 0.9f;
  color[1] = 0.9f;
  color[2] = 0.9f;
  viz.addPolyDataDisplay(mesh, color);
  viz2.addPolyDataDisplay(mesh, color);

  // Display surface normals
  if (DISPLAY_NORMALS)
  {
    color[0] = 0.9f;
    color[1] = 0.1f;
    color[2] = 0.1f;
    vtkSmartPointer<vtkPolyData> normals_data = vtkSmartPointer<vtkPolyData>::New();
    normals_data = planner.getInput();
    viz.addPolyNormalsDisplay(normals_data, color, scale);
  }

  // Display results without extra rasters
  tool_path_planner::ToolPathsData paths_no_extras_data = tool_path_planner::toToolPathsData(paths_no_extras.get());
  for (std::size_t i = 0; i < paths_no_extras_data.size(); ++i)
  {
    for (std::size_t j = 0; j < paths_no_extras_data[i].size(); ++j)
    {
      if (DISPLAY_LINES)  // display line
      {
        color[0] = 0.2f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths_no_extras_data[i][j].line, color, scale);
      }

      if (DISPLAY_DERIVATIVES)  // display derivatives
      {
        color[0] = 0.9f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz.addPolyNormalsDisplay(paths_no_extras_data[i][j].derivatives, color, scale);
      }
    }
  }

  viz.renderDisplay();

  // Display results with extra rasters
  tool_path_planner::ToolPathsData paths_with_extras_data = tool_path_planner::toToolPathsData(paths_with_extras.get());
  for (std::size_t i = 0; i < paths_with_extras_data.size(); ++i)
  {
    for (std::size_t j = 0; j < paths_with_extras_data[i].size(); ++j)
    {
      if (DISPLAY_LINES)  // display line
      {
        color[0] = 0.2f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz2.addPolyNormalsDisplay(paths_with_extras_data[i][j].line, color, scale);
      }

      if (DISPLAY_DERIVATIVES)  // display derivatives
      {
        color[0] = 0.9f;
        color[1] = 0.9f;
        color[2] = 0.2f;
        viz2.addPolyNormalsDisplay(paths_with_extras_data[i][j].derivatives, color, scale);
      }
    }
  }

  viz2.renderDisplay();
#endif
}

/*
TEST(IntersectTest, SurfaceWalkRasterRotationTest)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh1();

  double raster_directions[] = { 0., 0.78, 1.57, 2.35 };

  for (auto direction : raster_directions)
  {
    // Set input tool data
    tool_path_planner::SurfaceWalkRasterGenerator planner;
    tool_path_planner::SurfaceWalkRasterGenerator::Config tool;
    tool.point_spacing = POINT_SPACING;
    tool.raster_spacing = 0.75;
    tool.tool_offset = 0.0;                // currently unused
    tool.intersection_plane_height = 0.2;  // 0.5 works best, not sure if this should be included in the tool
    tool.min_hole_size = 0.1;
    tool.raster_rot_offset = direction;
    tool.debug = false;
    planner.setConfiguration(tool);
    runRasterRotationTest(planner, mesh);
  }
}

TEST(IntersectTest, PlaneSlicerRasterRotationTest)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh1();

  double raster_directions[] = { 0., 0.78, 1.57, 2.35 };

  for (auto direction : raster_directions)
  {
    // Set input tool data
    tool_path_planner::PlaneSlicerRasterGenerator planner;
    tool_path_planner::PlaneSlicerRasterGenerator::Config tool;
    tool.point_spacing = POINT_SPACING;
    tool.raster_spacing = 0.75;
    //    tool.tool_offset = 0.0;                // currently unused
    tool.min_hole_size = 0.1;
    tool.raster_rot_offset = direction;
    //    tool.debug = false;
    planner.setConfiguration(tool);
    runRasterRotationTest(planner, mesh);
  }
}

TEST(IntersectTest, HalfedgeEdgeGeneratorTestCase0)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh1();
  tool_path_planner::HalfedgeEdgeGenerator planner;
  tool_path_planner::HalfedgeEdgeGenerator::Config config;
  config.min_num_points = 20;
  config.point_dist = 0.1;
  planner.setConfiguration(config);
  runRasterRotationTest(planner, mesh);
}

TEST(IntersectTest, EigenValueEdgeGeneratorTestCase0)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh1();
  tool_path_planner::EigenValueEdgeGenerator planner;
  tool_path_planner::EigenValueEdgeGenerator::Config config;
  config.octree_res = 0.2;
  config.search_radius = 0.5;
  config.neighbor_tol = 0.9;
  planner.setConfiguration(config);
  runRasterRotationTest(planner, mesh);
}

// This test shows the results of the tool path planner on a square grid that has a sinusoidal
// variation in the z axis and a cutout in the middle.  It will generate a series of evenly spaced lines (aprox. equal
// to line_spacing) with evenly spaced points on each line (aprox. equal to pt_spacing).  Yellow arrows show the
// direction of travel in a give line (all lines should point in the same direction) and green arrows show the process
// normal direction (should be aprox. normal to the surface in that region)

TEST(IntersectTest, SurfaceWalkTestCase1)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh1();

  // Set input tool data
  tool_path_planner::SurfaceWalkRasterGenerator planner;
  tool_path_planner::SurfaceWalkRasterGenerator::Config tool;
  tool.point_spacing = POINT_SPACING;
  tool.raster_spacing = 0.75;
  tool.tool_offset = 0.0;                // currently unused
  tool.intersection_plane_height = 0.2;  // 0.5 works best, not sure if this should be included in the tool
  tool.min_hole_size = 0.1;
  tool.debug = false;
  planner.setConfiguration(tool);

  runTestCase1(planner, mesh);
}

TEST(IntersectTest, PlaneSlicerTestCase1)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh1();

  // Set input tool data
  tool_path_planner::PlaneSlicerRasterGenerator planner;
  tool_path_planner::PlaneSlicerRasterGenerator::Config tool;
  tool.point_spacing = POINT_SPACING;
  tool.raster_spacing = 0.75;
  //  tool.tool_offset = 0.0; // currently unused
  tool.min_hole_size = 0.1;
  //  tool.debug = false;
  planner.setConfiguration(tool);

  runTestCase1(planner, mesh);
}

TEST(IntersectTest, SurfaceWalkTestCaseRansac)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh2();

  // Set input tool data
  tool_path_planner::SurfaceWalkRasterGenerator planner;
  tool_path_planner::SurfaceWalkRasterGenerator::Config tool;
  tool.point_spacing = POINT_SPACING;
  tool.raster_spacing = 0.75;
  tool.tool_offset = 0.0;                // currently unused
  tool.intersection_plane_height = 0.2;  // 0.5 works best, not sure if this should be included in the tool
  tool.min_hole_size = 0.1;
  tool.debug = false;
  planner.setConfiguration(tool);
  runTestCaseRansac(planner, mesh);
}

TEST(IntersectTest, PlaneSlicerTestCaseRansac)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh2();

  // Set input tool data
  tool_path_planner::PlaneSlicerRasterGenerator planner;
  tool_path_planner::PlaneSlicerRasterGenerator::Config tool;
  tool.point_spacing = POINT_SPACING;
  tool.raster_spacing = 0.75;
  //  tool.tool_offset = 0.0; // currently unused
  tool.min_hole_size = 0.1;
  //  tool.debug = false;
  planner.setConfiguration(tool);
  runTestCaseRansac(planner, mesh);
}

TEST(IntersectTest, HalfedgeEdgeGeneratorTestRansac)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh2();
  tool_path_planner::HalfedgeEdgeGenerator planner;
  tool_path_planner::HalfedgeEdgeGenerator::Config config;
  config.min_num_points = 20;
  config.point_dist = 0.1;
  planner.setConfiguration(config);
  runTestCase1(planner, mesh);
}

TEST(IntersectTest, EigenValueEdgeGeneratorTestRansac)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh2();
  tool_path_planner::EigenValueEdgeGenerator planner;
  //  tool_path_planner::PlaneSlicerRasterGenerator::Config tool;
  //  planner.setConfiguration(tool);
  runTestCase1(planner, mesh);
}

TEST(IntersectTest, SurfaceWalkExtraRasterTest)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh1();

  tool_path_planner::SurfaceWalkRasterGenerator planner;
  tool_path_planner::SurfaceWalkRasterGenerator planner_with_extra;

  // Set input tool data
  tool_path_planner::SurfaceWalkRasterGenerator::Config tool;
  tool.point_spacing = POINT_SPACING;
  tool.raster_spacing = 0.75;
  tool.tool_offset = 0.0;                // currently unused
  tool.intersection_plane_height = 0.2;  // 0.5 works best, not sure if this should be included in the tool
  tool.min_hole_size = 0.1;
  tool.raster_rot_offset = 0.0;
  tool.generate_extra_rasters = false;
  tool.debug = false;

  planner.setConfiguration(tool);

  tool.generate_extra_rasters = true;
  planner_with_extra.setConfiguration(tool);

  runExtraRasterTest(planner, planner_with_extra, mesh);
}
*/
TEST(PlaneSlicerTest, PlaneSlicerExtraWaypointTest)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh1();

  tool_path_planner::PlaneSlicerRasterGenerator planner;
  tool_path_planner::PlaneSlicerRasterGenerator planner_with_extra;

  // Set input tool data
  tool_path_planner::PlaneSlicerRasterGenerator::Config tool;
  tool.raster_spacing = 1.0;
  tool.point_spacing = POINT_SPACING;
  tool.raster_rot_offset = 0.0;
  tool.min_segment_size = 0.05;
  tool.search_radius = 0.05;
  tool.min_hole_size = 0.8;
  //  tool.raster_wrt_global_axes = use:: tool_path_planner::PlaneSlicerRasterGenerator::DEFAULT_RASTER_WRT_GLOBAL_AXES;
  tool.raster_direction = Eigen::Vector3d::UnitY();
  tool.generate_extra_rasters = false;
  //  tool.raster_style = use:: tool_path_planner::PlaneSlicerRasterGenerator::KEEP_ORIENTATION_ON_REVERSE_STROKES;

  planner.setConfiguration(tool);

  tool.generate_extra_rasters = true;
  planner_with_extra.setConfiguration(tool);

  runExtraRasterTest(planner, planner_with_extra, mesh);
}

TEST(PlaneSlicerTest, PlaneSlicerExtraWaypointTest2)
{
  vtkSmartPointer<vtkPolyData> mesh = createTestMesh2();

  tool_path_planner::PlaneSlicerRasterGenerator planner;
  tool_path_planner::PlaneSlicerRasterGenerator planner_with_extra;

  // Set input tool data
  tool_path_planner::PlaneSlicerRasterGenerator::Config tool;
  tool.raster_spacing = 1.0;
  tool.point_spacing = POINT_SPACING;
  tool.raster_rot_offset = 0.0;
  tool.min_segment_size = 0.05;
  tool.search_radius = 0.05;
  tool.min_hole_size = 0.8;

  tool.raster_direction = Eigen::Vector3d::UnitY();
  tool.generate_extra_rasters = false;
  //  tool.raster_style = use:: tool_path_planner::PlaneSlicerRasterGenerator::KEEP_ORIENTATION_ON_REVERSE_STROKES;

  planner.setConfiguration(tool);

  tool.generate_extra_rasters = true;
  planner_with_extra.setConfiguration(tool);

  runExtraRasterTest(planner, planner_with_extra, mesh);

  mesh = loadTempMesh();

  tool.raster_spacing = .1;
  tool.point_spacing = .025;
  tool.raster_rot_offset = 0.0;
  tool.min_segment_size = 0.05;
  tool.search_radius = 0.05;
  tool.min_hole_size = 0.008;
  tool.raster_direction = Eigen::Vector3d::UnitY();
  tool.generate_extra_rasters = false;
  //  tool.raster_wrt_global_axes = use:: tool_path_planner::PlaneSlicerRasterGenerator::DEFAULT_RASTER_WRT_GLOBAL_AXES;
  //  tool.raster_style = use:: tool_path_planner::PlaneSlicerRasterGenerator::KEEP_ORIENTATION_ON_REVERSE_STROKES;
  planner.setConfiguration(tool);

  tool.generate_extra_rasters = true;
  planner_with_extra.setConfiguration(tool);

  runExtraRasterTest(planner, planner_with_extra, mesh, 0.1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
