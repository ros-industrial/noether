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
  VTKViewer::VTKViewer() //HWND hwnd
  {
    // Similar to Examples/Tutorial/Step1/Cxx/Cone.cxx
    // We create the basic parts of a pipeline and connect them
    this->renderer = vtkRenderer::New();
    this->renWin = vtkRenderWindow::New();
    this->renWin->AddRenderer(this->renderer);
    // setup the parent window
    //this->renWin->SetParentId(hwnd);
    this->iren = vtkRenderWindowInteractor::New();
    this->iren->SetRenderWindow(this->renWin);


    this->renderer->SetBackground(0.2,0.4,0.3);
    this->renWin->SetSize(800,800);
    // Finally we start the interactor so that event will be handled


  }

  void VTKViewer::renderDisplay()
  {
    this->renWin->Render();
    this->iren->Start();
  }

  void VTKViewer::addPointDataDisplay(vtkSmartPointer<vtkPoints> &points, std::vector<float> color)
  {
    // Add the grid points to a polydata object
      vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
      polydata->SetPoints(points);

      vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
      glyphFilter->SetInputConnection(polydata->GetProducerPort());
    #else
      glyphFilter->SetInputData(polydata);
    #endif
      glyphFilter->Update();

      // Create a mapper and actor
      vtkSmartPointer<vtkPolyDataMapper> pointsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      pointsMapper->SetInputData(glyphFilter->GetOutput());
      this->_poly_mappers.push_back(pointsMapper);

      vtkSmartPointer<vtkActor> pointsActor =
        vtkSmartPointer<vtkActor>::New();
      pointsActor->SetMapper(_poly_mappers.back());
      pointsActor->GetProperty()->SetPointSize(20);
      pointsActor->GetProperty()->SetColor(color[0],color[1],color[2]);

      this->_actors.push_back(pointsActor);

      // Add actor to renderer
      this->renderer->AddActor(_actors.back());

  }

  void VTKViewer::addPolyDataDisplay(vtkSmartPointer<vtkPolyData> &polydata , std::vector<float> color)
  {
    // create mapper and add to list
    vtkSmartPointer<vtkPolyDataMapper> triangulatedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    triangulatedMapper->SetInputData(polydata);
    this->_poly_mappers.push_back(triangulatedMapper);

    // create actor and add to list
    vtkSmartPointer<vtkActor> triangulatedActor = vtkSmartPointer<vtkActor>::New();
    triangulatedActor->SetMapper(_poly_mappers.back());
    triangulatedActor->GetProperty()->SetColor(color[0],color[1],color[2]);

    this->_actors.push_back(triangulatedActor);

    // Add actor to renderer
    this->renderer->AddActor(_actors.back());
  }

  VTKViewer::~VTKViewer()
  {
    renWin->Delete();
    renderer->Delete();
    iren->Delete();
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

    this->_poly_mappers.push_back(Mapper);

    // create actor and add to list
    vtkSmartPointer<vtkActor> triangulatedActor = vtkSmartPointer<vtkActor>::New();
    triangulatedActor->SetMapper(_poly_mappers.back());
    triangulatedActor->GetProperty()->SetColor(color[0],color[1],color[2]);

    this->_actors.push_back(triangulatedActor);

    // Add actor to renderer
    this->renderer->AddActor(_actors.back());
  }

}

