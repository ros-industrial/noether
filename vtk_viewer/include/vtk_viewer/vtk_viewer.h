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


    /**
     * @brief addPolyDataDisplay Add a renderer and actor for a polydata object (for meshes)
     * @param polydata The polydata to be displayed
     * @param color The color to use for rendering the data
     */
    void addPolyDataDisplay(vtkSmartPointer<vtkPolyData> &polydata, std::vector<float> color);

    /**
     * @brief addPolyNormalsDisplay Add a renderer and actor for a polydata object with normals (for lines with normals and derivatives)
     * @param polydata The polydata to be displayed
     * @param color The color to use for rendering the data
     * @param glyph The object used for displaying (usually arrows)
     */
    void addPolyNormalsDisplay(vtkSmartPointer<vtkPolyData> polydata, std::vector<float> color, vtkSmartPointer<vtkGlyph3D> glyph);

    /**
     * @brief addPointDataDisplay Add a renderer and actor for a point data object
     * @param points The point data to be displayed
     * @param color The color to use for rendering the data
     */
    void addPointDataDisplay(vtkSmartPointer<vtkPoints> &points, std::vector<float> color);

    /**
     * @brief addCellNormalDisplay Displays the normals for a mesh object
     * @param polydata The mesh object to be displayed
     * @param color The color to use for rendering the data
     */
    void addCellNormalDisplay(vtkSmartPointer<vtkPolyData> polydata, std::vector<float> color);

    /**
     * @brief renderDisplay Calls the VTK window Render() command to visualize all of the renderers added
     */
    void renderDisplay();

  private:

    vtkRenderWindow * renWin_; /**< The VTK window for displaying data */
    vtkRenderer * renderer_;  /**< The renderer for drawing all of the data in the display */
    vtkRenderWindowInteractor * iren_;  /**< The interactor for the display to capture mouse/keyboard input */

    std::vector<vtkSmartPointer<vtkActor> > actors_;  /**< The list of actors to add to the renderer */
    std::vector<vtkSmartPointer<vtkPolyDataMapper> > poly_mappers_;  /**< The list of mappers which loads polydata into the actors for displaying */

    /**
     * @brief MakeGlyphs  Creates a glyph object for displaying polydata (arrows)
     * @param src The input data to display
     * @param reverseNormals Flag to determine if the normals need to be flipped before displaying
     * @param glyph The pointer to the vtkGlyph3D object which will be created and returned
     */
    void MakeGlyphs(vtkSmartPointer<vtkPolyData>& src, bool const & reverseNormals, vtkSmartPointer<vtkGlyph3D> glyph);
  };

}
#endif // VTK_VIEWER_H
