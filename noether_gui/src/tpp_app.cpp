#include <noether_gui/widgets/tpp_widget.h>

#include <boost/program_options.hpp>
#include <boost_plugin_loader/plugin_loader.h>
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

  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(NOETHER_GUI_PLUGINS);
  loader.search_libraries_env = NOETHER_GUI_PLUGIN_LIBS_ENV;
  loader.search_paths_env = NOETHER_GUI_PLUGIN_PATHS_ENV;

  // Create the main window
  QMainWindow w;

  // Create (and optionally configure) the TPP widget
  auto* widget = new noether::TPPWidget(std::move(loader), &w);
  if (!mesh_file.empty())
    widget->setMeshFile(QString::fromStdString(mesh_file));
  if (!config_file.empty())
    widget->setConfigurationFile(QString::fromStdString(config_file));

  // Set the TPP widget as the central widget and show
  w.setCentralWidget(widget);
  w.show();

  return app.exec();
}
