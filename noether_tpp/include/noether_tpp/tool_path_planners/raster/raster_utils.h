#ifndef NOETHER_RASTER_UTILS_H
#define NOETHER_RASTER_UTILS_H

#include <noether_tpp/core/types.h>

#include <vector>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/PolygonMesh.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkKdTreePointLocator.h>

namespace noether
{
/**
 * @brief Inserts normals into the provided segment by averaging normals from nearby mesh points
 * @param search_radius Radius to search for nearby points
 * @param mesh_data_ Mesh data containing normals
 * @param kd_tree_ Kd-tree for efficient nearest neighbor search
 * @param segment_data Polydata defining raster segment to insert normals into
 * @return True if normals were successfully inserted, false otherwise
 */
bool insertNormals(const double search_radius,
                   vtkSmartPointer<vtkPolyData>& mesh_data_,
                   vtkSmartPointer<vtkKdTreePointLocator>& kd_tree_,
                   vtkSmartPointer<vtkPolyData>& segment_data);

/**
 * @brief Converts a pcl::PolygonMesh to a vtkSmartPointer<vtkPolyData> and computes normals if necessary.
 * @param mesh Input PCL PolygonMesh
 * @return vtkSmartPointer<vtkPolyData> containing the converted mesh with normals computed if necessary
 */
vtkSmartPointer<vtkPolyData> convertMeshToVTKPolyData(const pcl::PolygonMesh& mesh);

}  // namespace noether

#endif  // NOETHER_RASTER_UTILS_H
