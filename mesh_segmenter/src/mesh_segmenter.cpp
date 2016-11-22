/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <vtkIdList.h>
#include <vtkDataArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>

#include <mesh_segmenter/mesh_segmenter.h>

namespace mesh_segmenter
{

void MeshSegmenter::setInputMesh(vtkSmartPointer<vtkPolyData> mesh)
{
  input_mesh_ = mesh;

  triangle_filter_ = vtkSmartPointer<vtkTriangleFilter>::New();
  triangle_filter_->SetInputData(input_mesh_);
  triangle_filter_->Update();
}

std::vector<vtkSmartPointer<vtkPolyData> > MeshSegmenter::getMeshSegments()
{
  vtkSmartPointer<vtkPolyData> input_copy = vtkSmartPointer<vtkPolyData>::New();
  input_copy->DeepCopy(input_mesh_);
  std::vector<vtkSmartPointer<vtkPolyData> > meshes;
  for(int i = 0; i < included_indices_.size(); ++i)
  {

    vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
    mesh->DeepCopy(input_mesh_);
    mesh->GetPoints()->Initialize();
    mesh->GetPolys()->Initialize();
    input_copy->GetCellData()->CopyNormalsOn();

    // Preallocate memory, this is NECESSARY or normal data is NOT copied
    mesh->GetCellData()->CopyAllocate(input_copy->GetCellData(), included_indices_[i]->GetNumberOfIds(), included_indices_[i]->GetNumberOfIds());
    mesh->GetPointData()->CopyAllocate(input_copy->GetPointData(), included_indices_[i]->GetNumberOfIds()*2, included_indices_[i]->GetNumberOfIds()*2);

    // copy cell data and normals
    mesh->CopyCells(input_copy, included_indices_[i]);

    meshes.push_back(mesh);
  }

  return meshes;
}

void MeshSegmenter::segmentMesh()
{
  vtkSmartPointer<vtkIdList> used_cells = vtkSmartPointer<vtkIdList>::New();
  int size = input_mesh_->GetCellData()->GetNumberOfTuples();

  used_cells->Allocate(size);

  included_indices_.clear();

  for(int i = 0; i < size; ++i)
  {
    // if current index is not used, perform segmentation with it and insert the results into the list
    if(used_cells->IsId(i) == -1)
    {
      vtkSmartPointer<vtkIdList> linked_cells = vtkSmartPointer<vtkIdList>::New();
      linked_cells = segmentMesh(i);

      // save indices found
      included_indices_.push_back(linked_cells);
      for(int j = 0; j < linked_cells->GetNumberOfIds(); ++j)
      {
        used_cells->InsertNextId(linked_cells->GetId(j));
      }
    }
  }
}

vtkSmartPointer<vtkIdList> MeshSegmenter::segmentMesh(int start_cell)
{
  // Create the links object
  vtkSmartPointer<vtkIdList> unused_cells = vtkSmartPointer<vtkIdList>::New();
  vtkSmartPointer<vtkIdList> used_cells = vtkSmartPointer<vtkIdList>::New();

  vtkDataArray* normals = input_mesh_->GetCellData()->GetNormals();

  if(!normals)
  {
    return used_cells;
  }

  // Insert the start cell
  unused_cells->InsertNextId(start_cell);

  // Loop and find all connected cells
  while(unused_cells->GetNumberOfIds())
  {
    vtkSmartPointer<vtkIdList> neighbors = vtkSmartPointer<vtkIdList>::New();
    neighbors = getNeighborCells(input_mesh_, unused_cells->GetId(0));

    // check to see if neighbors are in the used/unused list
    for(int i = 0; i < neighbors->GetNumberOfIds(); ++i)
    {

      // if a cell has not been seen, check the angle to determine if it should be added to the unused list
      if(unused_cells->IsId(neighbors->GetId(i)) == -1 && used_cells->IsId(neighbors->GetId(i)) == -1)
      {
        //check angle, insert if it is valid
        const double* const n1 = normals->GetTuple(unused_cells->GetId(0));
        // vtk does not return a const ptr, need to make a copy before getting next pointer
        double val[3] = {n1[0], n1[1], n1[2]};
        const double* const n2 = normals->GetTuple(neighbors->GetId(i));
        if( areNormalsNear(&val[0], n2, 0.3) )
        {
          unused_cells->InsertNextId(neighbors->GetId(i));
        }
      }
    }

    // Once all adjacent cells have been checked, move the first unused_cell to the used_cell list
    used_cells->InsertNextId(unused_cells->GetId(0));
    unused_cells->DeleteId(unused_cells->GetId(0));
  }

  // return id list
  return used_cells;
}

vtkSmartPointer<vtkIdList> MeshSegmenter::getNeighborCells(vtkSmartPointer<vtkPolyData> mesh, int cellId)
{
  //vtkIdType cellId = 0;

  vtkSmartPointer<vtkIdList> cellPointIds = vtkSmartPointer<vtkIdList>::New();
  triangle_filter_->GetOutput()->GetCellPoints(cellId, cellPointIds);

  vtkSmartPointer<vtkIdList> neighbors = vtkSmartPointer<vtkIdList>::New();

  for(vtkIdType i = 0; i < cellPointIds->GetNumberOfIds(); i++)
  {
    vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();

    //add one of the edge points
    idList->InsertNextId(cellPointIds->GetId(i));

    //add the other edge point
    idList->InsertNextId(cellPointIds->GetId((i+1) % (cellPointIds->GetNumberOfIds() )  ));

    //get the neighbors of the cell
    vtkSmartPointer<vtkIdList> neighborCellIds = vtkSmartPointer<vtkIdList>::New();

    triangle_filter_->GetOutput()->GetCellNeighbors(cellId, idList, neighborCellIds);
    for(vtkIdType j = 0; j < neighborCellIds->GetNumberOfIds(); j++)
    {
      neighbors->InsertNextId(neighborCellIds->GetId(j));
    }

  }
  return neighbors;
}

// The VTK Polydata::CopyData() function does not appear to copy the point and cell data (normals), this is the
// function that will perform it as expected
void MeshSegmenter::extractMeshData(vtkSmartPointer<vtkIdList> ids, vtkSmartPointer<vtkPolyData> output_mesh)
{
   vtkSmartPointer<vtkDoubleArray> pt_norms = vtkSmartPointer<vtkDoubleArray>::New();
   pt_norms->SetNumberOfComponents(3);
   vtkSmartPointer<vtkDoubleArray> cell_norms = vtkSmartPointer<vtkDoubleArray>::New();
   cell_norms->SetNumberOfComponents(3);

   for(int i = 0; i < ids->GetNumberOfIds(); ++i)
   {
      //pt_norms->InsertNextTuple(input_mesh_->)
   }
}

bool MeshSegmenter::areNormalsNear(const double* norm1, const double* norm2, double threshold)
{
  double val = (norm1[0]*norm2[0] + norm1[1]*norm2[1] + norm1[2]*norm2[2]);
  if( (val) >= 1.0) // acos(1.0) can sometimes result in NaN, check and return true
  {
    return true;
  }
  return ( acos(val) <= threshold );
}

}
