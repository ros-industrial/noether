#! /bin/bash
source /opt/noether/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jvm/java-11-openjdk-amd64/lib:/usr/lib/jvm/java-11-openjdk-amd64/lib/server
./opt/noether/install/noether_gui/bin/noether_gui_app
