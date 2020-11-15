/**
 * @file mouse_interactor.h
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

  /**
   * @brief setSaveLocation Set the pathway for saving polydata to
   * @param pathway The pathway to save to
   */
  void setSaveLocation(std::string pathway) { save_location_.str(pathway); }

  /**
   * @brief getSaveLocation Get the current pathway used to save polydata to
   * @return The current pathway for saving data
   */
  std::string getSaveLocation() { return save_location_.str(); }

  /**
   * @brief OnLeftButtonDown Callback function for handling left mouse button click events, selects an actor in a window
   */
  virtual void OnLeftButtonDown();

  /**
   * @brief OnKeyPress Callback to handle keyboard events, pressing 'a' will save the selected polydata to the
   * 'save_location_'
   */
  virtual void OnKeyPress();

  vtkSmartPointer<vtkActor> selected_actor_; /**< the actor that was clicked on in the window */

  std::stringstream save_location_; /**< location to save polydata files to */
};

}  // namespace vtk_viewer
#endif  // MOUSE_INTERACTOR_H
