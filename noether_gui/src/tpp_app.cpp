#include <noether_gui/widgets/tpp_widget.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <QApplication>
#include <signal.h>

void handleSignal(int /*sig*/) { QApplication::instance()->quit(); }

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  boost_plugin_loader::PluginLoader loader;
  loader.search_libraries.insert(NOETHER_GUI_PLUGINS);
  loader.search_libraries_env = NOETHER_GUI_PLUGIN_LIBS_ENV;
  loader.search_paths_env = NOETHER_GUI_PLUGIN_PATHS_ENV;

  noether::TPPWidget w(std::move(loader));
  w.show();

  return app.exec();
}
