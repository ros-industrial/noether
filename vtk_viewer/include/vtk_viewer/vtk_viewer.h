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
#include <vtkSmartPointer.h>
#include <vtk_viewer/mouse_interactor.h>

namespace vtk_viewer
{


  class VTKViewer
  {
  public:

    VTKViewer();
    ~VTKViewer();


    /**
     * @brief addPolyDataDisplay Add a renderer and actor for a polydata object
     * (for meshes)
     * @param polydata The polydata to be displayed
     * @param color The color to use for rendering the data
     */
    void addPolyDataDisplay(vtkPolyData* polydata, std::vector<float> color);

    /**
     * @brief addPolyNormalsDisplay Add a renderer and actor for a polydata
     * object with normals (for lines with normals and derivatives)
     * @param polydata The polydata to be displayed
     * @param color The color to use for rendering the data
     * @param scale The size to scale and show the arrows at
     */
    void addPolyNormalsDisplay(vtkPolyData* polydata, std::vector<float> color, double scale);

    /**
     * @brief addPointDataDisplay Add a renderer and actor for a point data object
     * @param points The point data to be displayed
     * @param color The color to use for rendering the data
     */
    void addPointDataDisplay(vtkPoints* points, std::vector<float> color);

    /**
     * @brief addCellNormalDisplay Displays the normals for a mesh object
     * @param polydata The mesh object to be displayed
     * @param color The color to use for rendering the data
     * @param scale The size to scale and show the arrows at
     */
    void addCellNormalDisplay(vtkPolyData* polydata, std::vector<float> color, double scale);

    /**
     * @brief renderDisplay Calls the VTK window Render() command to visualize
     * all of the renderers added
     */
    void renderDisplay();

    /**
     * @brief getNumberOfDisplayObjects Get the number of actors, and thus the number of
     * objects currently being displayed
     * @return The number of actor objects
     */
    int getNumberOfDisplayObjects(){return actors_.size();}

    /**
     * @brief removeObjectDisplay Remove an object from the list of objects
     * being displayed
     * @param index The index of the item to be removed
     * @return True if the index of the item to remove exists and was removed,
     * false if the index exceeds the list of items available
     */
    bool removeObjectDisplay(int index);

    /**
     * @brief removeAllDisplays Removes all displays from the renderer
     */
    void removeAllDisplays();

    /**
     * @brief setLogDir Set the directory for saving polydata files to
     * @param dir The directory to save data to
     */
    void setLogDir(std::string dir){mouse_interactor_->setSaveLocation(dir);}

    /**
     * @brief getLogDir Get the directory used for saving polydata files to
     * @return The directory currently used for saving data
     */
    std::string getLogDir(){return mouse_interactor_->getSaveLocation();}

  private:

    vtkRenderWindow * renWin_; /**< The VTK window for displaying data */
    vtkRenderer * renderer_;  /**< The renderer for drawing all of the data in the display */
    vtkRenderWindowInteractor * iren_;  /**< The interactor for the display to capture mouse/keyboard input */
    MouseInteractorStyle* mouse_interactor_; /** The custom interactor style which provides additional options to save polydata to a log directory */

    std::vector<vtkSmartPointer<vtkActor> > actors_;  /**< The list of actors to add to the renderer */
    std::vector<vtkSmartPointer<vtkPolyDataMapper> > poly_mappers_;  /**< The list of mappers which loads polydata into the actors for displaying */

    /**
     * @brief MakeGlyphs  Creates a glyph object for displaying polydata (arrows)
     * @param src The input data to display
     * @param reverseNormals Flag to determine if the normals need to be flipped before displaying
     * @param glyph The pointer to the vtkGlyph3D object which will be created and returned
     * @param scale The size to scale and show the arrows at
     */
    void makeGlyphs(vtkPolyData* src, bool const & reverseNormals, vtkSmartPointer<vtkGlyph3D> glyph, double scale);
  };

}
#endif // VTK_VIEWER_H
