/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include "vtk_viewer/vtk_viewer.h"
#include <vtkProperty.h>
#include <vtkVertexGlyphFilter.h>

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
}

