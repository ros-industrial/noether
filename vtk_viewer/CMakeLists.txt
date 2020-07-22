cmake_minimum_required(VERSION 3.5.0)
project(vtk_viewer VERSION 0.1.0)

find_package(cmake_common_scripts REQUIRED)

find_package(VTK REQUIRED NO_MODULE)
if(VTK_FOUND AND ("${VTK_VERSION}" VERSION_LESS 7.1))
  message(FATAL_ERROR "The minimum required version of VTK is 7.1, but found ${VTK_VERSION}")
  set(VTK_FOUND FALSE)
endif()

find_package(PCL REQUIRED)
if(PCL_FOUND AND ("${PCL_VERSION}" VERSION_LESS 1.9))
  message(WARNING "The minimum required version of PCL is 1.9, but found ${PCL_VERSION} in path first. Checking for exactly 1.9")
  find_package(PCL REQUIRED 1.9)
else()
  add_definitions(${PCL_DEFINITIONS})
endif()

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)

if(NOETHER_ENABLE_TESTING)
    set(CLANG_ARGS ${DEFAULT_CLANG_TIDY_ERROR_ARGS})
else()
    set(CLANG_ARGS ${DEFAULT_CLANG_TIDY_WARNING_ARGS})
endif()

add_code_coverage()

add_library(${PROJECT_NAME} SHARED src/mouse_interactor.cpp src/vtk_utils.cpp src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES} ${VTK_LIBRARIES} console_bridge)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION 14)
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${CLANG_ARGS} ENABLE ${NOETHER_ENABLE_TESTING})
target_include_what_you_use(${PROJECT_NAME} ARGUMENTS ${DEFAULT_IWYU_ARGS} ENABLE ${NOETHER_ENABLE_TESTING})
target_cppcheck(${PROJECT_NAME} ARGUMENTS ${DEFAULT_CPPCHECK_ARGS} ENABLE ${NOETHER_ENABLE_TESTING})

configure_package(NAMESPACE noether TARGETS ${PROJECT_NAME})

# Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if (NOETHER_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target()
  add_subdirectory(test)
endif()
