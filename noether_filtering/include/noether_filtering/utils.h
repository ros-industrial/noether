/*
 * utils.h
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_NOETHER_FILTERING_UTILS_H_
#define INCLUDE_NOETHER_FILTERING_UTILS_H_

#include <cxxabi.h>
#include <memory>
#include <string>
#include <typeinfo>

#include <pcl/PolygonMesh.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

namespace noether_filtering
{
namespace utils
{
template <class C>
static std::string getClassName()
{
  int status = -4;  // some arbitrary value to eliminate the compiler warning
  const char* mangled_name = typeid(C).name();

  // enable c++11 by passing the flag -std=c++11 to g++
  std::unique_ptr<char, void (*)(void*)> res{ abi::__cxa_demangle(mangled_name, NULL, NULL, &status), std::free };
  return (status == 0) ? res.get() : mangled_name;
}

/**
 * @brief vtk2TriangleMesh - pcl's vtk2mesh function can produce a PolygonMesh containing 1-point
 * or 2-point 'polygons', as well as squares and so on.  Many applications assume a mesh containing
 * only triangles.  This function converts the output to only include triangles.
 * @param poly_data - input - the VTK mesh
 * @param mesh - output - the resulting pcl polygonmesh containing only triangles
 * @return - number of points contained in pcl::PolygonMesh, as vtk2mesh does.
 */
int vtk2TriangleMesh(const vtkSmartPointer<vtkPolyData>& poly_data, pcl::PolygonMesh& mesh);

}  // namespace utils
}  // namespace noether_filtering

#endif /* INCLUDE_NOETHER_FILTERING_UTILS_H_ */
