#include <noether_gui/widgets/tpp_widget.h>
#include <noether_gui/plugin_interface.h>

#include <boost/program_options.hpp>
#include <QApplication>
#include <QMainWindow>
#include <signal.h>

void handleSignal(int /*sig*/) { QApplication::instance()->quit(); }

int main(int argc, char** argv)
{
  namespace po = boost::program_options;

  std::string mesh_file;
  std::string config_file;

  po::options_description desc("options");
  // clang-format off
  desc.add_options()
      ("help", "produce help message")
      ("mesh,m", po::value<std::string>(&mesh_file), "Input mesh file")
      ("config,c", po::value<std::string>(&config_file), "Tool path planning pipeline configuration file")
  ;
  // clang-format on
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  QApplication app(argc, argv);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  auto factory = std::make_shared<noether::GuiFactory>();

  // Create (and optionally configure) the TPP widget
  auto widget = noether::TPPWidget(factory);
  widget.setWindowIcon(QIcon(":/icons/icon.jpg"));
  widget.setWindowTitle("Tool Path Planner");
  widget.showMaximized();

  if (!mesh_file.empty())
    widget.setMeshFile(QString::fromStdString(mesh_file));
  if (!config_file.empty())
    widget.setConfigurationFile(QString::fromStdString(config_file));

  return app.exec();
}
