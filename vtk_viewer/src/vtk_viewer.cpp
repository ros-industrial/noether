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


#define VTK_SP(type, name)\
  vtkSmartPointer<type> name = vtkSmartPointer<type>::New()

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

    this->renderer_->SetBackground(0.2,0.4,0.3);
    this->renWin_->SetSize(1200,1000);
  }

  void VTKViewer::renderDisplay()
  {
    this->renWin_->Render();
    this->iren_->Start();
  }

  void VTKViewer::addPointDataDisplay(vtkSmartPointer<vtkPoints> &points, std::vector<float> color)
  {
    // Add the grid points to a polydata object
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polydata);
    glyphFilter->Update();

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> pointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    pointsMapper->SetInputData(glyphFilter->GetOutput());
    this->poly_mappers_.push_back(pointsMapper);

    vtkSmartPointer<vtkActor> pointsActor =
      vtkSmartPointer<vtkActor>::New();
    pointsActor->SetMapper(poly_mappers_.back());
    pointsActor->GetProperty()->SetPointSize(20);
    pointsActor->GetProperty()->SetColor(color[0],color[1],color[2]);

    this->actors_.push_back(pointsActor);

    // Add actor to renderer
    this->renderer_->AddActor(actors_.back());

  }

  void VTKViewer::addPolyDataDisplay(vtkSmartPointer<vtkPolyData> &polydata , std::vector<float> color)
  {
    // create mapper and add to list
    vtkSmartPointer<vtkPolyDataMapper> triangulatedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    triangulatedMapper->SetInputData(polydata);
    this->poly_mappers_.push_back(triangulatedMapper);

    // create actor and add to list
    vtkSmartPointer<vtkActor> triangulatedActor = vtkSmartPointer<vtkActor>::New();
    triangulatedActor->SetMapper(poly_mappers_.back());
    triangulatedActor->GetProperty()->SetColor(color[0],color[1],color[2]);

    this->actors_.push_back(triangulatedActor);

    // Add actor to renderer
    this->renderer_->AddActor(actors_.back());
  }

  VTKViewer::~VTKViewer()
  {
    renWin_->Delete();
    renderer_->Delete();
    iren_->Delete();
  }

  void VTKViewer::MakeGlyphs(vtkSmartPointer<vtkPolyData>& src, bool const & reverseNormals , vtkSmartPointer<vtkGlyph3D> glyph)
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
    glyph->SetScaleFactor(1);
    glyph->SetColorModeToColorByVector();
    glyph->SetScaleModeToScaleByVector();
    glyph->OrientOn();
    glyph->Update();
  }

  void VTKViewer::addPolyNormalsDisplay(vtkSmartPointer<vtkPolyData> &polydata, std::vector<float> color, vtkSmartPointer<vtkGlyph3D> glyph)
  {
    MakeGlyphs(polydata, true, glyph);

    vtkSmartPointer<vtkPolyDataMapper> Mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    Mapper->SetInputData(glyph->GetOutput());
    Mapper->SetScalarModeToUsePointFieldData();

    this->poly_mappers_.push_back(Mapper);

    // create actor and add to list
    vtkSmartPointer<vtkActor> triangulatedActor = vtkSmartPointer<vtkActor>::New();
    triangulatedActor->SetMapper(poly_mappers_.back());
    triangulatedActor->GetProperty()->SetColor(color[0],color[1],color[2]);

    this->actors_.push_back(triangulatedActor);

    // Add actor to renderer_
    this->renderer_->AddActor(actors_.back());
  }

}

