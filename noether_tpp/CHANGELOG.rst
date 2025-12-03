^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package noether_tpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Copied header data from input mesh into primitive fit meshes (`#346 <https://github.com/marip8/noether/issues/346>`_)
* Contributors: Michael Ripperger

0.15.1 (2025-12-01)
-------------------
* Fixed undefined references to template functions in the factory classes (`#345 <https://github.com/marip8/noether/issues/345>`_)
* Contributors: Michael Ripperger

0.15.0 (2025-12-01)
-------------------
* Updates to support ``boost_plugin_loader`` 0.4.3 (`#331 <https://github.com/marip8/noether/issues/331>`_)
* Fix vertex normal vector normalization in `NormalsFromMeshFaces` mesh modifier (`#342 <https://github.com/marip8/noether/issues/342>`_)
* Created mesh face subdivision mesh modifiers (`#341 <https://github.com/marip8/noether/issues/341>`_)
* Updated cylinder primitive generation to create uniform triangles along cylinder body (`#340 <https://github.com/marip8/noether/issues/340>`_)
* Updated the mesh modifier unit test (`#338 <https://github.com/marip8/noether/issues/338>`_)
* Updated utilities for accessing data in ``pcl::PCLPointCloud2`` (`#337 <https://github.com/marip8/noether/issues/337>`_)
* Refactored implementation of `NormalsFromMeshFaces` mesh modifier (`#334 <https://github.com/marip8/noether/issues/334>`_)
* Refactored RANSAC-based shape primitive fitting mesh modifiers (`#336 <https://github.com/marip8/noether/issues/336>`_)
* Changed ``PlaneProjection`` mesh modifier plugin name to `RansacPlaneProjection`
* Changed ``CylinderSegmentation`` mesh modifier plugin name to `RansacCylinderProjection`
* Contributors: David Spielman, DavidMerzJr, Michael Ripperger, Tyler Marr

0.14.0 (2025-11-17)
-------------------
* Added capability for cylinder primitive generation (`#329 <https://github.com/marip8/noether/issues/329>`_)
* Normalized computed axes to prevent floating point rounding error from propogating through the tpp pipeline (`#330 <https://github.com/marip8/noether/issues/330>`_)
* Created RANSAC-based cylinder projection mesh modifier (`#328 <https://github.com/marip8/noether/issues/328>`_)
* Contributors: David Spielman, Michael Ripperger

0.13.0 (2025-10-07)
-------------------
* Added ability to generate shape primitive meshes (plane and ellipsoid) via command line executable (`#326 <https://github.com/marip8/noether/issues/326>`_)
* Contributors: David Spielman, Michael Ripperger

0.12.0 (2025-09-24)
-------------------
* Moved ``printException`` utility from ``noether_gui`` to ``noether_tpp`` (`#320 <https://github.com/marip8/noether/issues/320>`_)
* Added ``boost_plugin_loader`` to ``noether_tpp`` CMake dependencies export (`#317 <https://github.com/marip8/noether/issues/317>`_)
* Updated CMakeLists to link against VTK Modules (`#313 <https://github.com/marip8/noether/issues/313>`_)
* Updated deprecated VTK functions ``GetRenderWindow`` and ``GetInteractor`` (`#312 <https://github.com/marip8/noether/issues/312>`_)
* Contributors: David Spielman, Michael Ripperger

0.11.0 (2025-09-16)
-------------------
* Added CMake format files and CI job (`#315 <https://github.com/marip8/noether/issues/315>`_)
* Created plugin interface and plugins for `noether_tpp` package (`#310 <https://github.com/marip8/noether/issues/310>`_)
* Updated ``noether_gui`` package to use ``noether_tpp`` plugins (`#314 <https://github.com/marip8/noether/issues/314>`_)
* Contributors: Michael Ripperger

0.10.0 (2025-08-01)
-------------------
* Added clang-tidy file; removed code coverage addition; removed test run target; set compile options in noether_tpp (`#311 <https://github.com/marip8/noether/issues/311>`_)
* Fix doxygen warnings (`#309 <https://github.com/marip8/noether/issues/309>`_)
* Added YAML serialization for `noether_tpp` (`#308 <https://github.com/marip8/noether/issues/308>`_)
* Contributors: Michael Ripperger

0.9.0 (2025-07-31)
------------------

0.8.0 (2025-07-14)
------------------
* Updated documentation (`#302 <https://github.com/marip8/noether/issues/302>`_)
* Updated plane projection modifier (`#293 <https://github.com/marip8/noether/issues/293>`_)
* Contributors: David Spielman, Michael Ripperger, snw1317

0.7.0 (2025-03-27)
------------------
* Updated to use ``pcl::io`` instead of ``pcl::VTKUtils`` for mesh conversions (`#294 <https://github.com/marip8/noether/issues/294>`_)
* Fixed bug in compound modifier introduced in `#283 <https://github.com/marip8/noether/issues/283>`_ (`#288 <https://github.com/marip8/noether/issues/288>`_)
* Fix error reporting around missing vertex normals (`#283 <https://github.com/marip8/noether/issues/283>`_)
* Updated documentation (`#282 <https://github.com/marip8/noether/issues/282>`_)
* Added biased tool drag tool path modifier (`#279 <https://github.com/marip8/noether/issues/279>`_)
* Contributors: David Spielman, Michael Ripperger

0.6.0 (2024-10-09)
------------------
* Minor updates (`#273 <https://github.com/marip8/noether/issues/273>`_)
* Added fill holes mesh modifier GUI (`#272 <https://github.com/marip8/noether/issues/272>`_)
* Added point spacing tool path modifiers and utilities (`#270 <https://github.com/marip8/noether/issues/270>`_)
* Add normal estimation mesh modifiers (`#268 <https://github.com/marip8/noether/issues/268>`_)
* Removed duplicate raster rectification occurring within plane slicer raster planner (`#266 <https://github.com/marip8/noether/issues/266>`_)
* Ported boundary edge tool path planner (`#267 <https://github.com/marip8/noether/issues/267>`_)
* Contributors: Michael Ripperger

0.5.1 (2024-09-27)
------------------
* Updated unit tests (`#265 <https://github.com/marip8/noether/issues/265>`_)
* Contributors: Michael Ripperger

0.5.0 (2024-09-09 11:06)
------------------------
* Updated behavior of tool drag orientation to split overshoot on both sides of path (`#251 <https://github.com/marip8/noether/issues/251>`_)
* Contributors: Michael Ripperger, Walter Glockner

0.4.0 (2024-09-09 10:21)
------------------------
* Fixed issue with Eigen size mismatch in doing calculation (`#258 <https://github.com/marip8/noether/issues/258>`_)
* Fully supported direction generator in plane slicer raster planner  (`#257 <https://github.com/marip8/noether/issues/257>`_)
* Updated plane slice raster planner to utilize direction generator; improved computation of number of cutting planes (`#255 <https://github.com/marip8/noether/issues/255>`_)
* Contributors: Michael Ripperger, Tyler Marr

0.3.0 (2024-08-08)
------------------
* Updated CI for Ubuntu Noble (`#250 <https://github.com/marip8/noether/issues/250>`_)
* Fixed BSpline reconstruction modifier (`#256 <https://github.com/marip8/noether/issues/256>`_)
* Simplified finding of VTK 7.1+ (`#254 <https://github.com/marip8/noether/issues/254>`_)
* Added cross hatch raster tool path planner (`#248 <https://github.com/marip8/noether/issues/248>`_)
* Added class for combining multiple tool path planners and concatenating their outputs (`#247 <https://github.com/marip8/noether/issues/247>`_)
* Removed copy and move constructors/operators from specific classes (`#244 <https://github.com/marip8/noether/issues/244>`_)
* Added offset tool path modifier (`#227 <https://github.com/marip8/noether/issues/227>`_)
* Checked number of polygons before attempting to project in place (`#236 <https://github.com/marip8/noether/issues/236>`_)
* Updated plane projection mesh modifier (`#233 <https://github.com/marip8/noether/issues/233>`_)
* Updated Euclidean clustering mesh modifier (`#232 <https://github.com/marip8/noether/issues/232>`_)
* Contributors: Humvee3982, Michael Ripperger

0.2.0 (2024-05-11)
------------------
* Updated planners to return empty tool paths vector before attempting to apply modifiers (`#230 <https://github.com/marip8/noether/issues/230>`_)
* Updated documentation (`#229 <https://github.com/marip8/noether/issues/229>`_)
* Added tool path modifier to perform concatentation (`#224 <https://github.com/marip8/noether/issues/224>`_)
* Created RANSAC-based plane projection mesh modifier (`#223 <https://github.com/marip8/noether/issues/223>`_)
* Fixed bug in linear approach/departure tool path modifiers (`#222 <https://github.com/marip8/noether/issues/222>`_)
* Contributors: David Spielman, Michael Ripperger

0.1.0 (2024-01-04)
------------------
* Updated for compatibility with Ubuntu Jammy (`#211 <https://github.com/marip8/noether/issues/211>`_)
* Adds missing PCL dependency (`#207 <https://github.com/marip8/noether/issues/207>`_)
* Created linear departure tool path modifier (`#202 <https://github.com/marip8/noether/issues/202>`_)
* Run clang-format job on 20.04 (`#201 <https://github.com/marip8/noether/issues/201>`_)
* Created linear approach tool path modifier (`#200 <https://github.com/marip8/noether/issues/200>`_)
* Fixed plane slicer raster planner planning with coplanar mesh. (`#197 <https://github.com/marip8/noether/issues/197>`_)
* Added colcon.pkg file to noether_tpp (`#192 <https://github.com/marip8/noether/issues/192>`_)
* Move classes to individual files (`#187 <https://github.com/marip8/noether/issues/187>`_)
* Added angle offset, lead in, and lead out tool path modifiers (`#183 <https://github.com/marip8/noether/issues/183>`_)
* Fixed normals in plane_slicer_raster_planner.cpp (`#180 <https://github.com/marip8/noether/issues/180>`_)
* Ported the plane slice raster tool path planner (`#168 <https://github.com/marip8/noether/issues/168>`_)
* Added compound mesh modifier (`#163 <https://github.com/marip8/noether/issues/163>`_)
* Added orientation smoothing tool path modifier (`#160 <https://github.com/marip8/noether/issues/160>`_)
* Defined pointer definitions in class (`#159 <https://github.com/marip8/noether/issues/159>`_)
* Added functions to set common parameters for edge and raster planners (`#162 <https://github.com/marip8/noether/issues/162>`_)
* Updated modifier default behavior (`#161 <https://github.com/marip8/noether/issues/161>`_)
* Refactored tool path planner unit test (`#155 <https://github.com/marip8/noether/issues/155>`_)
* Refactored tool path modifiers for standard raster and edge path planners (`#154 <https://github.com/marip8/noether/issues/154>`_)
* Created organization tool path modifiers (`#153 <https://github.com/marip8/noether/issues/153>`_)
* Created raster direction/origin generators (`#152 <https://github.com/marip8/noether/issues/152>`_)
* Introduced interfaces for repository re-design (`#151 <https://github.com/marip8/noether/issues/151>`_)
* Contributors: David Spielman, DavidMerzJr, Iñigo Moreno, Mark Johnson, Michael Ripperger, Tom Noble, Tyler Marr

0.0.2 (2021-07-19)
------------------

0.0.1 (2020-10-12)
------------------
