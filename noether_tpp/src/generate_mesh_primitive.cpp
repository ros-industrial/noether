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
                           "\"ellipsoid\" \n");
  pcl::console::print_info(" where options are:\n");
  pcl::console::print_info("                     -x_dim X           = the x dimension of the mesh primitive. "
                           "(default: ");
  pcl::console::print_value("%f", 1.0);
  pcl::console::print_info(")\n");
  pcl::console::print_info("                     -y_dim X           = the y dimension of the mesh primitive. "
                           "(default: ");
  pcl::console::print_value("%f", 1.0);
  pcl::console::print_info(")\n");
  pcl::console::print_info("                     -a_dim X           = the dimension of the principle semi-axis along "
                           "the x-axis. "
                           "(default: ");
  pcl::console::print_value("%f", 2.0);
  pcl::console::print_info(")\n");
  pcl::console::print_info("                     -b_dim X           = the dimension of the principle semi-axis along "
                           "the y-axis. "
                           "(default: ");
  pcl::console::print_value("%f", 2.0);
  pcl::console::print_info(")\n");
  pcl::console::print_info("                     -c_dim X           = the dimension of the principle semi-axis along "
                           "the z-axis. "
                           "(default: ");
  pcl::console::print_value("%f", 1.0);
  pcl::console::print_info(")\n");
  pcl::console::print_info("                     -resolution X           = defines the resolution of the ellipsoid. "
                           "The longitudes will be split into resolution segments (i.e. there are resolution + 1 "
                           "latitude lines including the north and south pole). The latitudes will be split into 2 * "
                           "resolution segments (i.e. there are 2 * resolution longitude lines.) "
                           "(default: ");
  pcl::console::print_value("%i", 20);
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
  if (primitive_type != "plane" && primitive_type != "ellipsoid")
  {
    pcl::console::print_error("Argument type can only be \"plane\" or \"ellipsoid\".\n");
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

  double a_dimension = 2.0;
  pcl::console::parse_argument(argc, argv, "-a_dim", a_dimension);
  if (a_dimension <= 0)
  {
    pcl::console::print_error("Argument -a_dim must be greater than zero.\n");
    return (-1);
  }

  double b_dimension = 2.0;
  pcl::console::parse_argument(argc, argv, "-b_dim", b_dimension);
  if (b_dimension <= 0)
  {
    pcl::console::print_error("Argument -b_dim must be greater than zero.\n");
    return (-1);
  }

  double c_dimension = 1.0;
  pcl::console::parse_argument(argc, argv, "-c_dim", c_dimension);
  if (c_dimension <= 0)
  {
    pcl::console::print_error("Argument -c_dim must be greater than zero.\n");
    return (-1);
  }

  int resolution = 20;
  pcl::console::parse_argument(argc, argv, "-resolution", resolution);
  if (resolution <= 0)
  {
    pcl::console::print_error("Argument -resolution must be greater than zero.\n");
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

  if (primitive_type == "ellipsoid")
  {
    output_mesh = noether::createEllipsoidMesh(a_dimension, b_dimension, c_dimension, resolution, tf);
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
