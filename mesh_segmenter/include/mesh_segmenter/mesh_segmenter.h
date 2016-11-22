/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#ifndef MESH_SEGMENTER_H
#define MESH_SEGMENTER_H

#include <vtk_viewer/vtk_utils.h>
#include <vtkPolyData.h>
#include <vtkTriangleFilter.h>

namespace mesh_segmenter
{
  class MeshSegmenter
  {
  public:

    void setInputMesh(vtkSmartPointer<vtkPolyData> mesh);

    vtkSmartPointer<vtkPolyData> getInputMesh(){return input_mesh_;}

    vtkSmartPointer<vtkIdList> getNeighborCells(vtkSmartPointer<vtkPolyData> mesh, int cellId);

    void segmentMesh();

    std::vector<vtkSmartPointer<vtkPolyData> > getMeshSegments();

    vtkSmartPointer<vtkIdList> segmentMesh(int start_cell);

    bool areNormalsNear(const double* norm1, const double* norm2, double threshold);

    void extractMeshData(vtkSmartPointer<vtkIdList> ids, vtkSmartPointer<vtkPolyData> output_mesh);
  private:
    vtkSmartPointer<vtkPolyData> input_mesh_;
    vtkSmartPointer<vtkTriangleFilter> triangle_filter_;
    std::vector<vtkSmartPointer<vtkIdList> > included_indices_;
  };
}


#endif // MESH_SEGMENTER_H
