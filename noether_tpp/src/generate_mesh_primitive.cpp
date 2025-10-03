#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <filesystem>
#include <noether_tpp/utils.h>
#include <yaml-cpp/yaml.h>
#include <noether_tpp/serialization.h>

void printHelp(char** argv)
{
  pcl::console::print_error("Syntax is: %s output_filepath type <options>\n", argv[0]);
  pcl::console::print_info(" where output_filepath = path in which to save the generated mesh \n");
  pcl::console::print_info("       type            = primitive shape to generate (Available options are \"plane\" and "
                           "\"half_ellipsoid\" \n");
  pcl::console::print_info(" where options are:\n");
  pcl::console::print_info("                     -x_dim X           = the x dimension of the mesh primitive. "
                           "(default: ");
  pcl::console::print_value("%f", 1.0);
  pcl::console::print_info(")\n");
  pcl::console::print_info("                     -y_dim X           = the y dimension of the mesh primitive. "
                           "(default: ");
  pcl::console::print_value("%f", 1.0);
  pcl::console::print_info(")\n");
  pcl::console::print_info("                     -tf X              = the transformation applied to each of the mesh. "
                           "vertices"
                           "(default: ");
  pcl::console::print_value("%s", "Eigen::Isometry3d::Identity()");
  pcl::console::print_info(")\n");
  pcl::console::print_info("Format for -tf is as follows: \n");
  pcl::console::print_value("%s",
                            "-tf \"x: 0.0 \n"
                            "y: 0.0 \n"
                            "z: 0.0 \n"
                            "qw: 0.0 \n"
                            "qx: 0.0 \n"
                            "qy: 0.0 \n"
                            "qz: 0.0\" \n");
  pcl::console::print_info("Where x,y,z are the translation components in meters and qw, qx, qy, qz are the quaternion "
                           "components that represent the rotation component of the transformation \n");
}

void printSuccess(char** argv)
{
  pcl::console::print_info("Generated a %s mesh and saved it to %s. \n"
                           "For more information on available primative types and mesh parameters, "
                           "use: %s -h or %s --help \n",
                           argv[2],
                           argv[1],
                           argv[0],
                           argv[0]);
}

int main(int argc, char** argv)
{
  if (argc < 3 || pcl::console::find_argument(argc, argv, "--help") > 0 ||
      pcl::console::find_argument(argc, argv, "-h") > 0)
  {
    printHelp(argv);
    return (-1);
  }

  // Parse command line arguments
  std::string output_file = argv[1];
  std::string output_extension = std::filesystem::path(output_file).extension().string();
  if (output_extension != ".stl" && output_extension != ".ply")
  {
    pcl::console::print_error("The only output file types supported are: .stl and .ply.\n");
    return (-1);
  }

  std::string primitive_type = argv[2];
  if (primitive_type != "plane" && primitive_type != "half_ellipsoid")
  {
    pcl::console::print_error("Argument type can only be \"plane\" or \"half_ellipsoid\".\n");
    return (-1);
  }

  double x_dimension = 1.0;
  pcl::console::parse_argument(argc, argv, "-x_dim", x_dimension);
  if (x_dimension <= 0)
  {
    pcl::console::print_error("Argument -x_dim must be greater than zero.\n");
    return (-1);
  }

  double y_dimension = 1.0;
  pcl::console::parse_argument(argc, argv, "-y_dim", y_dimension);
  if (y_dimension <= 0)
  {
    pcl::console::print_error("Argument -y_dim must be greater than zero.\n");
    return (-1);
  }

  // Convert tf argument from string to EigenIsometry3d using defined YAML serialization
  std::string tf_string;
  pcl::console::parse_argument(argc, argv, "-tf", tf_string);
  Eigen::Isometry3d tf{ Eigen::Isometry3d::Identity() };
  if (!tf_string.empty())
  {
    YAML::Node tf_yaml;
    try
    {
      tf_yaml = YAML::Load(tf_string);
    }
    catch (const YAML::Exception& e)
    {
      pcl::console::print_error(e.what());
    }
    tf = tf_yaml.as<Eigen::Isometry3d>();
  }

  pcl::PolygonMesh output_mesh;
  if (primitive_type == "plane")
  {
    output_mesh = noether::createPlaneMesh(x_dimension, y_dimension, tf);
  }

  if (primitive_type == "half_ellipsoid")
  {
    pcl::console::print_error("Half ellipsoid generation is not implemented yet. \n");
    return -1;
  }

  // Save the generated mesh to file
  if (output_extension == ".ply")
  {
    if (pcl::io::savePolygonFilePLY(output_file, output_mesh))
    {
      printSuccess(argv);
      return 0;
    }
    else
    {
      pcl::io::savePolygonFilePLY(output_file, output_mesh);
      printHelp(argv);
      return -1;
    }
  }
  if (output_extension == ".stl")
  {
    if (pcl::io::savePolygonFileSTL(output_file, output_mesh))
    {
      pcl::io::savePolygonFileSTL(output_file, output_mesh);
      printSuccess(argv);
      return 0;
    }
    else
    {
      pcl::io::savePolygonFileSTL(output_file, output_mesh);
      printHelp(argv);
      return -1;
    }
  }
}
