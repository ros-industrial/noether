#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkRendererCollection.h>
#include <vtkDataSetMapper.h>
#include <vtkUnstructuredGrid.h>
#include <vtkIdTypeArray.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPlaneSource.h>
#include <vtkCellPicker.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkSelectionNode.h>
#include <vtkSelection.h>
#include <vtkExtractSelection.h>
#include <vtkObjectFactory.h>

#include <boost/filesystem.hpp>
#include <vtkXMLPolyDataWriter.h>
#include <vtk_viewer/mouse_interactor.h>

namespace vtk_viewer
{
vtkStandardNewMacro(MouseInteractorStyle);

void MouseInteractorStyle::OnKeyPress()
{
  vtkRenderWindowInteractor* rwi = this->Interactor;
  std::string key = rwi->GetKeySym();

  // Handle a "normal" key
  if (key == "a")
  {
    // check to see if the pointer exists;
    if (selected_actor_)
    {
      vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
      vtkPolyData* pd = vtkPolyData::SafeDownCast(selected_actor_->GetMapper()->GetInput());

      // check to make sure that there is data to save
      if (pd->GetNumberOfCells() > 0 || pd->GetPoints()->GetNumberOfPoints() > 0)
      {
        writer->SetInputData(pd);

        // Save the data if the directory exists and the file can be created
        if (boost::filesystem::is_directory(save_location_.str().c_str()))
        {
          int count = 0;
          std::stringstream polydata_file;
          polydata_file << save_location_.str().c_str() << "/polydata" << count << ".vtp";

          while (std::ifstream(polydata_file.str().c_str()) && count < 100)
          {
            std::cout << "File already exists" << std::endl;
            ++count;
            polydata_file.str("");
            polydata_file << save_location_.str().c_str() << "/polydata" << count << ".vtp";
          }

          std::ofstream file(polydata_file.str().c_str());
          if (!file)
          {
            std::cout << "File " << polydata_file.str().c_str() << " could not be created" << std::endl;
          }
          else
          {
            std::cout << "Saving polydata file: " << polydata_file.str().c_str() << "\n";
            file.close();
            writer->SetFileName(polydata_file.str().c_str());
            writer->Write();
          }
        }
        else
        {
          std::cout << "Directory " << save_location_.str().c_str() << " does not exist.  Not saving polydata file."
                    << std::endl;
        }
      }
    }
  }

  // Forward events
  vtkInteractorStyleTrackballCamera::OnKeyPress();
}  // end OnKeyPress

void MouseInteractorStyle::OnLeftButtonDown()
{
  // Get the location of the click (in window coordinates)
  int* pos = this->GetInteractor()->GetEventPosition();

  vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
  picker->SetTolerance(0.0005);

  // Pick from this location.
  picker->Pick(pos[0], pos[1], 0, this->GetDefaultRenderer());

  double* worldPosition = picker->GetPickPosition();

  if (picker->GetCellId() != -1)
  {
    vtkSmartPointer<vtkIdTypeArray> ids = vtkSmartPointer<vtkIdTypeArray>::New();
    ids->SetNumberOfComponents(1);
    ids->InsertNextValue(picker->GetCellId());

    selected_actor_ = picker->GetActor();
  }
  else
  {
    // selected_actor_ = NULL;
  }
  // Forward events
  vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}  // end OnLeftButtonDown

}  // namespace vtk_viewer
