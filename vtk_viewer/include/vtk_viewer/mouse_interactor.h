#ifndef MOUSE_INTERACTOR_H
#define MOUSE_INTERACTOR_H

#include <sstream>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkDataSetMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkAbstractMapper3D.h>
#include <vtkInteractorStyleTrackballCamera.h>

namespace vtk_viewer
{


class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static MouseInteractorStyle* New();

    MouseInteractorStyle()
    {
      selected_actor_ = vtkSmartPointer<vtkActor>::New();
      save_location_.str("");
    }

    void setSaveLocation(std::string pathway){save_location_.str(pathway);}
    std::string getSaveLocation(){return save_location_.str().c_str();}

    virtual void OnLeftButtonDown();
    virtual void OnKeyPress();

    vtkSmartPointer<vtkPolyData> Data;
    vtkSmartPointer<vtkActor> selected_actor_;

    std::stringstream save_location_;
};



}
#endif // MOUSE_INTERACTOR_H
