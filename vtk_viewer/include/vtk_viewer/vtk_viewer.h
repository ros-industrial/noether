/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#ifndef VTK_VIEWER_H
#define VTK_VIEWER_H

#include "vtkConeSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include <vtkPolyData.h>
#include <vtkGlyph3D.h>

namespace vtk_viewer
{


  class VTKViewer
  {
  public:
    VTKViewer();
    ~VTKViewer();

    void addPolyDataDisplay(vtkSmartPointer<vtkPolyData> &polydata, std::vector<float> color);
    void addPolyNormalsDisplay(vtkSmartPointer<vtkPolyData> &polydata, std::vector<float> color, vtkSmartPointer<vtkGlyph3D> glyph);
    void addPointDataDisplay(vtkSmartPointer<vtkPoints> &points, std::vector<float> color);
    void renderDisplay();

  private:
    vtkRenderWindow *renWin;
    vtkRenderer *renderer;
    vtkRenderWindowInteractor *iren;

    std::vector<vtkSmartPointer<vtkActor> > _actors;
    std::vector<vtkSmartPointer<vtkPolyDataMapper> >_poly_mappers;

    void MakeGlyphs(vtkSmartPointer<vtkPolyData>& src, bool const & reverseNormals, vtkSmartPointer<vtkGlyph3D> glyph);
  };

}
#endif // VTK_VIEWER_H
