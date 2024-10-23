#! /bin/bash
source /opt/noether/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$LD_LIBRARY_PATH_ADDITIONS
./opt/noether/install/noether_gui/bin/noether_gui_app
