^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package noether_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.15.0 (2025-12-01)
-------------------
* Updated to ``boost_plugin_loader`` version 0.4.3 and removed plugin cache from factory objects (`#331 <https://github.com/marip8/noether/issues/331>`_)
* Created face subdivision mesh modifier classes (`#341 <https://github.com/marip8/noether/issues/341>`_)
* Updated cylinder primitive generation to create uniform triangles along cylinder body (`#340 <https://github.com/marip8/noether/issues/340>`_)
* Revised shape primitive fitting mesh modifiers (`#336 <https://github.com/marip8/noether/issues/336>`_)
* Changed ``PlaneProjection`` mesh modifier plugin name to `RansacPlaneProjection`
* Changed ``CylinderSegmentation`` mesh modifier plugin name to `RansacCylinderProjection`
* Contributors: David Spielman, Michael Ripperger, Tyler Marr

0.14.0 (2025-11-17)
-------------------
* Created a mesh modifier that projects vertices onto a fitted cylinder shape primitive (`#328 <https://github.com/marip8/noether/issues/328>`_)
* Contributors: Michael Ripperger

0.13.0 (2025-10-07)
-------------------
* Handle negative values in unit spin boxes (`#327 <https://github.com/marip8/noether/issues/327>`_)
* Contributors: Michael Ripperger

0.12.0 (2025-09-24)
-------------------
* Updated configurable TPP pipeline widget to use QSettings to save last opened/saved configuration files (`#325 <https://github.com/marip8/noether/issues/325>`_)
* Added simplified GUI plugin macro and created pluign FAQ section in documentation (`#322 <https://github.com/marip8/noether/issues/322>`_)
* Added checks before widget deletion to prevent warning messages (`#323 <https://github.com/marip8/noether/issues/323>`_)
* Moved printException utility from noether_gui to noether_tpp (`#320 <https://github.com/marip8/noether/issues/320>`_)
* Handled case of widget plugin configuration failing (`#319 <https://github.com/marip8/noether/issues/319>`_)
* Updated deprecated VTK functions GetRenderWindow and GetInteractor (`#312 <https://github.com/marip8/noether/issues/312>`_)
* Contributors: David Spielman, Michael Ripperger

0.11.0 (2025-09-16)
-------------------
* Added plugin interface and plugins for ``noether_tpp`` package (`#310 <https://github.com/marip8/noether/issues/310>`_)
* Updated GUI plugin interface and plugins (`#314 <https://github.com/marip8/noether/issues/314>`_)
* GUI plugins no longer create `noether_tpp` components; they create the configuration files for the new `noether_tpp` component plugins.
* Added CMake format files and CI job (`#315 <https://github.com/marip8/noether/issues/315>`_)
* Contributors: Michael Ripperger

0.10.0 (2025-08-01)
-------------------
* Added clang-tidy file; removed code coverage addition; removed test run target; set compile options in noether_tpp (`#311 <https://github.com/marip8/noether/issues/311>`_)
* Fix doxygen warnings (`#309 <https://github.com/marip8/noether/issues/309>`_)
* Contributors: Michael Ripperger

0.9.0 (2025-07-31)
------------------
* Replaced splitter with dock widget in tool path planning widget (`#307 <https://github.com/marip8/noether/issues/307>`_)
* Added double spin box for angle values with unit support (`#305 <https://github.com/marip8/noether/issues/305>`_, `#306 <https://github.com/marip8/noether/issues/306>`_)
* Minor updates to GUI interfaces (`#303 <https://github.com/marip8/noether/issues/303>`_)
* Contributors: Michael Ripperger

0.8.0 (2025-07-14)
------------------
* Updated documentation (`#302 <https://github.com/marip8/noether/issues/302>`_)
* Added double spin box for distance values with units (`#301 <https://github.com/marip8/noether/issues/301>`_)
* Added button to GUI to save tool path as a YAML file (`#296 <https://github.com/marip8/noether/issues/296>`_)
* Added origin axis display to GUI (`#300 <https://github.com/marip8/noether/issues/300>`_)
* Added default config file directory to config tpp pipline widget (`#297 <https://github.com/marip8/noether/issues/297>`_)
* Appending .yaml extension to file when saving tpp config (`#289 <https://github.com/marip8/noether/issues/289>`_)
* Updated plane projection modifier (`#293 <https://github.com/marip8/noether/issues/293>`_)
* Added feature to GUI to save modified meshes (`#291 <https://github.com/marip8/noether/issues/291>`_)
* Contributors: David Spielman, Eugenio Bernardi, Michael Ripperger, snw1317

0.7.0 (2025-03-27)
------------------
* Updated to use pcl::io instead of pcl::VTKUtils for mesh conversions (`#294 <https://github.com/marip8/noether/issues/294>`_)
* Fixed error reporting around missing vertex normals (`#283 <https://github.com/marip8/noether/issues/283>`_)
* Updated documentation (`#282 <https://github.com/marip8/noether/issues/282>`_)
* Added biased tool drag toolpath modifier (`#279 <https://github.com/marip8/noether/issues/279>`_)
* Contributors: David Spielman, Michael Ripperger

0.6.0 (2024-10-09)
------------------
* Store loaded plugins internally in TPP pipeline widget (`#275 <https://github.com/marip8/noether/issues/275>`_)
* Minor updates (`#273 <https://github.com/marip8/noether/issues/273>`_)
* Add fill holes mesh modifier GUI (`#272 <https://github.com/marip8/noether/issues/272>`_)
* Created point spacing tool path modifiers and utilities (`#270 <https://github.com/marip8/noether/issues/270>`_)
* Add normal estimation mesh modifiers (`#268 <https://github.com/marip8/noether/issues/268>`_)
* Ported boundary edge tool path planner (`#267 <https://github.com/marip8/noether/issues/267>`_)
* Contributors: Michael Ripperger

0.5.1 (2024-09-27)
------------------
* Fixed plugin loader widget to prevent plugin libraries from being unloaded (`#264 <https://github.com/marip8/noether/issues/264>`_)
* Remove forward declaration, include yaml.h, link with yaml-cpp (`#263 <https://github.com/marip8/noether/issues/263>`_)
* Updated tool button text in case icon graphics do not exist (`#260 <https://github.com/marip8/noether/issues/260>`_)
* Contributors: Michael Ripperger, Douglas Smith

0.5.0 (2024-09-09 11:06)
------------------------

0.4.0 (2024-09-09 10:21)
------------------------
* Optionally get `bidirectional` parameter in `PlaneSliceRasterPlanner` to maintain backwards compatibility (`#259 <https://github.com/marip8/noether/issues/259>`_)
* Fully support direction generator in plane slicer raster planner  (`#257 <https://github.com/marip8/noether/issues/257>`_)
* Contributors: Michael Ripperger

0.3.0 (2024-08-08)
------------------
* Updated CI for Ubuntu Noble (`#250 <https://github.com/marip8/noether/issues/250>`_)
* Installed executables for access by ROS (`#253 <https://github.com/marip8/noether/issues/253>`_)
* Various minor updates to the GUI (`#249 <https://github.com/marip8/noether/issues/249>`_, `#246 <https://github.com/marip8/noether/issues/246>`_)
* Created cross hatch raster tool path planner (`#248 <https://github.com/marip8/noether/issues/248>`_)
* Removed collapsible area widget (`#245 <https://github.com/marip8/noether/issues/245>`_)
* Revised plugin loader widget for better usability (`#239 <https://github.com/marip8/noether/issues/239>`_)
* Removed unnecessary MOC of widgets (`#238 <https://github.com/marip8/noether/issues/238>`_)
* Updated default display of tool path waypoints to be the modified tool path (`#237 <https://github.com/marip8/noether/issues/237>`_)
* Created an offset tool path modifier (`#227 <https://github.com/marip8/noether/issues/227>`_)
* Updated GUI layout for improved UX  (`#228 <https://github.com/marip8/noether/issues/228>`_)
* Added display to show tool path as polyline (`#226 <https://github.com/marip8/noether/issues/226>`_)
* Updated plane projection mesh modifier (`#233 <https://github.com/marip8/noether/issues/233>`_)
* Updated Euclidean clustering mesh modifier (`#232 <https://github.com/marip8/noether/issues/232>`_)
* Contributors: Humvee3982, Michael Ripperger

0.2.0 (2024-05-11)
------------------
* Colorized mesh fragments in visualization (`#231 <https://github.com/marip8/noether/issues/231>`_)
* Updated documentation (`#229 <https://github.com/marip8/noether/issues/229>`_)
* Created tool path modifier to perform concatenation (`#224 <https://github.com/marip8/noether/issues/224>`_)
* Created RANSAC-based plane projection mesh modifier (`#223 <https://github.com/marip8/noether/issues/223>`_)
* Create configurable TPP pipeline widget (`#212 <https://github.com/marip8/noether/issues/212>`_)
* Contributors: David Spielman, Michael Ripperger

0.1.0 (2024-01-04)
------------------
* Updated for compatibility with Ubuntu Jammy (`#211 <https://github.com/marip8/noether/issues/211>`_)
* Export dependencies of noether_gui (`#209 <https://github.com/marip8/noether/issues/209>`_)
* Added ability for user to toggle visibility of applied tool path and mesh modifiers (`#208 <https://github.com/marip8/noether/issues/208>`_)
* Created linear departure tool path modifier (`#202 <https://github.com/marip8/noether/issues/202>`_)
* Run clang-format job on 20.04 (`#201 <https://github.com/marip8/noether/issues/201>`_)
* Created linear approach tool path modifier (`#200 <https://github.com/marip8/noether/issues/200>`_)
* Added viewer to TPP pipeline application (`#191 <https://github.com/marip8/noether/issues/191>`_)
* Updated inclusion of YAML headers (`#194 <https://github.com/marip8/noether/issues/194>`_)
* Move classes to individual files (`#187 <https://github.com/marip8/noether/issues/187>`_)
* Added compile definitions for plugin loader environment variables (`#186 <https://github.com/marip8/noether/issues/186>`_)
* Add YAML Configuration for TPP GUI (`#184 <https://github.com/marip8/noether/issues/184>`_)
* Added angle offset, lead in, and lead out tool path modifiers (`#183 <https://github.com/marip8/noether/issues/183>`_)
* Fixed logic in GUI exception (`#171 <https://github.com/marip8/noether/issues/171>`_)
* Created a GUI Package (`#169 <https://github.com/marip8/noether/issues/169>`_)
* Contributors: David Spielman, Michael Ripperger, Tyler Marr

0.0.2 (2021-07-19)
------------------

0.0.1 (2020-10-12)
------------------
