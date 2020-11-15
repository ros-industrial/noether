/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */
#include "vtk_viewer/vtk_viewer.h"
#include <vtkProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkReverseSense.h>
#include <vtkMaskPoints.h>
#include <vtkArrowSource.h>

#include <vtkLookupTable.h>
#include <vtkColorTransferFunction.h>
#include <vtkColorSeries.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkDoubleArray.h>
#include <vtkCell.h>
#include <vtkTriangle.h>

#include <vtk_viewer/mouse_interactor.h>

#define VTK_SP(type, name) vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

namespace vtk_viewer
{
VTKViewer::VTKViewer()
{
  // We create the basic parts of a pipeline and connect them
  this->renderer_ = vtkRenderer::New();
  this->renWin_ = vtkRenderWindow::New();
  this->renWin_->AddRenderer(this->renderer_);

  // setup the parent window
  this->iren_ = vtkRenderWindowInteractor::New();
  this->iren_->SetRenderWindow(this->renWin_);

  // Add custom interactor style
  mouse_interactor_ = MouseInteractorStyle::New();
  mouse_interactor_->SetDefaultRenderer(renderer_);
  iren_->SetInteractorStyle(mouse_interactor_);

  this->renderer_->SetBackground(0.2, 0.4, 0.3);
  this->renWin_->SetSize(1200, 1000);
}

void VTKViewer::renderDisplay()
{
  this->renWin_->Render();
  this->iren_->Start();
}

void VTKViewer::addPointDataDisplay(vtkPoints* points, const std::vector<float>& color)
{
  // Add the grid points to a polydata object
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  vtkSmartPointer<vtkVertexGlyphFilter> glyph_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyph_filter->SetInputData(polydata);
  glyph_filter->Update();

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> points_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  points_mapper->SetInputData(glyph_filter->GetOutput());
  this->poly_mappers_.push_back(points_mapper);

  vtkSmartPointer<vtkActor> points_actor = vtkSmartPointer<vtkActor>::New();
  points_actor->SetMapper(poly_mappers_.back());
  points_actor->GetProperty()->SetPointSize(20);
  points_actor->GetProperty()->SetColor(
      static_cast<double>(color[0]), static_cast<double>(color[1]), static_cast<double>(color[2]));

  this->actors_.push_back(points_actor);

  // Add actor to renderer
  this->renderer_->AddActor(actors_.back());
}

void VTKViewer::addPolyDataDisplay(vtkPolyData* polydata, const std::vector<float>& color)
{
  // create mapper and add to list
  vtkSmartPointer<vtkPolyDataMapper> triangulated_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangulated_mapper->SetInputData(polydata);
  this->poly_mappers_.push_back(triangulated_mapper);

  // create actor and add to list
  vtkSmartPointer<vtkActor> triangulated_actor = vtkSmartPointer<vtkActor>::New();
  triangulated_actor->SetMapper(poly_mappers_.back());
  triangulated_actor->GetProperty()->SetColor(
      static_cast<double>(color[0]), static_cast<double>(color[1]), static_cast<double>(color[2]));

  this->actors_.push_back(triangulated_actor);

  // Add actor to renderer
  this->renderer_->AddActor(actors_.back());
}

VTKViewer::~VTKViewer()
{
  renWin_->Delete();
  renderer_->Delete();
  iren_->Delete();
}

void VTKViewer::makeGlyphs(vtkPolyData* src,
                           bool const& reverseNormals,
                           vtkSmartPointer<vtkGlyph3D> glyph,
                           double scale)
{
  // Sometimes the contouring algorithm can create a volume whose gradient
  // vector and ordering of polygon(using the right hand rule) are
  // inconsistent. vtkReverseSense cures this problem.
  VTK_SP(vtkReverseSense, reverse);
  VTK_SP(vtkMaskPoints, maskPts);
  maskPts->SetOnRatio(1);  // number of arrows to skip for visualizing, 1 means no skipping
  maskPts->RandomModeOn();
  if (reverseNormals)
  {
    reverse->SetInputData(src);
    reverse->ReverseCellsOn();
    reverse->ReverseNormalsOn();
    maskPts->SetInputConnection(reverse->GetOutputPort());
  }
  else
  {
    maskPts->SetInputData(src);
  }

  // Source for the glyph filter
  VTK_SP(vtkArrowSource, arrow);
  arrow->SetTipResolution(5);
  arrow->SetTipLength(0.3);
  arrow->SetTipRadius(0.1);

  glyph->SetSourceConnection(arrow->GetOutputPort());
  glyph->SetInputConnection(maskPts->GetOutputPort());
  glyph->SetVectorModeToUseNormal();
  glyph->SetScaleFactor(scale);
  glyph->SetColorModeToColorByVector();
  glyph->SetScaleModeToScaleByVector();
  glyph->OrientOn();
  glyph->Update();
}

void VTKViewer::addPolyNormalsDisplay(vtkPolyData* polydata, const std::vector<float>& color, double scale)
{
  VTK_SP(vtkGlyph3D, glyph);
  makeGlyphs(polydata, false, glyph, scale);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(glyph->GetOutput());
  mapper->SetScalarModeToUsePointFieldData();

  this->poly_mappers_.push_back(mapper);

  // create actor and add to list
  vtkSmartPointer<vtkActor> triangulated_actor = vtkSmartPointer<vtkActor>::New();
  triangulated_actor->SetMapper(poly_mappers_.back());
  triangulated_actor->GetProperty()->SetColor(
      static_cast<double>(color[0]), static_cast<double>(color[1]), static_cast<double>(color[2]));

  this->actors_.push_back(triangulated_actor);

  // Add actor to renderer_
  this->renderer_->AddActor(actors_.back());
}

void VTKViewer::addCellNormalDisplay(vtkPolyData* polydata, const std::vector<float>& color, double scale)
{
  // get cell and point data

  vtkSmartPointer<vtkPoints> points = polydata->GetPoints();
  vtkSmartPointer<vtkPoints> cell_centroids = vtkSmartPointer<vtkPoints>::New();

  vtkSmartPointer<vtkDataArray> normals_array = polydata->GetCellData()->GetNormals();

  vtkSmartPointer<vtkCellArray> cell_ids = polydata->GetPolys();
  cell_ids->InitTraversal();

  for (int i = 0; i < cell_ids->GetNumberOfCells(); ++i)
  {
    vtkCell* cell = polydata->GetCell(i);
    if (cell)
    {
      vtkTriangle* triangle = dynamic_cast<vtkTriangle*>(cell);
      double p0[3];
      double p1[3];
      double p2[3];
      double center[3];
      triangle->GetPoints()->GetPoint(0, p0);
      triangle->GetPoints()->GetPoint(1, p1);
      triangle->GetPoints()->GetPoint(2, p2);
      triangle->TriangleCenter(p0, p1, p2, center);
      cell_centroids->InsertNextPoint(center);
    }
  }
  vtkSmartPointer<vtkPolyData> centroid_polydata = vtkSmartPointer<vtkPolyData>::New();
  centroid_polydata->SetPoints(cell_centroids);
  centroid_polydata->GetPointData()->SetNormals(normals_array);

  addPolyNormalsDisplay(centroid_polydata, color, scale);
}

bool VTKViewer::removeObjectDisplay(std::size_t index)
{
  if (index >= actors_.size())
  {
    return false;
  }

  // First, remove the desired actor from the renderer
  renderer_->RemoveActor(actors_[index]);

  // Delete the actor then the mapper associated with the data
  actors_.erase(actors_.begin() + static_cast<long>(index));
  poly_mappers_.erase(poly_mappers_.begin() + static_cast<long>(index));
  return true;
}

void VTKViewer::removeAllDisplays()
{
  while (actors_.size() > 0)
  {
    renderer_->RemoveActor(actors_.back());
    actors_.erase(actors_.end() - 1);
    poly_mappers_.erase(poly_mappers_.end() - 1);
  }
}
}  // namespace vtk_viewer
