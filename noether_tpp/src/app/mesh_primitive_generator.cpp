#include <boost/program_options.hpp>
#include <iostream>
#include <pcl/io/auto_io.h>

#include <noether_tpp/utils.h>

int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::options_description opts("Mesh primitive generation options");

  // clang-format off
  opts.add_options()
    ("help,h", "Produce help message")
    ("type,t", po::value<std::string>()->default_value("plane")->required(), "Primitive type")
    ("out_file,o", po::value<std::string>()->required(), "Output mesh file");

  // Plane options
  po::options_description plane_opts("Plane options");
  plane_opts.add_options()
    ("lx", po::value<float>()->default_value(1.0f), "Length (m) in the x direction")
    ("ly", po::value<float>()->default_value(1.0f), "Width (m) in the y direction");

  // Ellipsoid options
  po::options_description ellipsoid_opts("Ellipsoid options");
  ellipsoid_opts.add_options()
    ("rx", po::value<float>()->default_value(1.0f), "Radius (m) in the x direction")
    ("ry", po::value<float>()->default_value(1.0f), "Radius (m) in the y direction")
    ("rz", po::value<float>()->default_value(1.5f), "Radius (m) in the z direction")
    ("resolution,r", po::value<int>()->default_value(20), "Number of points in each ring of the ellipsoid")
    ("theta", po::value<float>()->default_value(180.0f), "Angle range (deg) of ellipsoid spanning from pole to pole, on (0, pi]")
    ("phi", po::value<float>()->default_value(360.0f), "Angle range (deg) of ellipsoid around z-axis passing through both poles, on (0, 2 * pi]");

  po::options_description origin_opts("Origin options");
  origin_opts.add_options()
    ("o.x", po::value<double>()->default_value(0.0), "Translation (m) in the x direction")
    ("o.y", po::value<double>()->default_value(0.0), "Translation (m) in the y direction")
    ("o.z", po::value<double>()->default_value(0.0), "Translation (m) in the z direction")
    ("o.rx", po::value<double>()->default_value(0.0), "Euler XYZ rotation (deg) about the x axis")
    ("o.ry", po::value<double>()->default_value(0.0), "Euler XYZ rotation (deg) about the y axis")
    ("o.rz", po::value<double>()->default_value(0.0), "Euler XYZ rotation (deg) about the z axis");
  // clang-format on

  // Add the primitive options to the general options
  opts.add(plane_opts).add(ellipsoid_opts).add(origin_opts);

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, opts), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << opts << std::endl;
    return 1;
  }

  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  if (vm.count("o.x"))
    origin.translation().x() = vm.at("o.x").as<double>();
  if (vm.count("o.y"))
    origin.translation().y() = vm.at("o.y").as<double>();
  if (vm.count("o.z"))
    origin.translation().z() = vm.at("o.z").as<double>();
  if (vm.count("o.rx"))
    origin.rotate(Eigen::AngleAxisd(vm.at("o.rx").as<double>() * M_PI / 180.0, Eigen::Vector3d::UnitX()));
  if (vm.count("o.ry"))
    origin.rotate(Eigen::AngleAxisd(vm.at("o.ry").as<double>() * M_PI / 180.0, Eigen::Vector3d::UnitY()));
  if (vm.count("o.rz"))
    origin.rotate(Eigen::AngleAxisd(vm.at("o.rz").as<double>() * M_PI / 180.0, Eigen::Vector3d::UnitZ()));

  pcl::PolygonMesh mesh;
  const auto type = vm.at("type").as<std::string>();
  if (type == "plane")
  {
    std::cout << "Generating plane mesh..." << std::endl;
    const auto lx = vm.at("lx").as<float>();
    const auto ly = vm.at("ly").as<float>();
    mesh = noether::createPlaneMesh(lx, ly, origin);
  }
  else if (type == "ellipsoid")
  {
    std::cout << "Generating ellipsoid mesh..." << std::endl;
    const auto rx = vm.at("rx").as<float>();
    const auto ry = vm.at("ry").as<float>();
    const auto rz = vm.at("rz").as<float>();
    const auto resolution = vm.at("resolution").as<int>();
    const auto theta_range = vm.at("theta").as<float>() * static_cast<float>(M_PI / 180.0);
    const auto phi_range = vm.at("phi").as<float>() * static_cast<float>(M_PI / 180.0);
    mesh = noether::createEllipsoidMesh(rx, ry, rz, resolution, theta_range, phi_range, origin);
  }
  else
  {
    std::cout << "Unsupported primitive type: '" << type << "'" << std::endl;
    return -1;
  }

  const std::string out_file = vm.at("out_file").as<std::string>();
  if (pcl::io::save(out_file, mesh) < 0)
  {
    std::cout << "Failed to save mesh to '" << out_file << "'" << std::endl;
    return -1;
  }
  else
  {
    std::cout << "Saved mesh to '" << out_file << "'" << std::endl;
  }

  return 1;
}
