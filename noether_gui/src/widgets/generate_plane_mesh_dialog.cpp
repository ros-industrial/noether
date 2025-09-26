#include <noether_gui/widgets/generate_plane_mesh_dialog.h>
#include "ui_generate_plane_mesh_dialog.h"

// QT
#include <QDoubleSpinBox>
#include <QDialog>
#include <QWidget>

namespace noether
{
GeneratePlaneMeshDialog::GeneratePlaneMeshDialog(QWidget* parent)
  : QDialog(parent), ui_(new Ui::GeneratePlaneMeshDialog)
{
  ui_->setupUi(this);
  setWindowTitle("Plane Mesh Dimensions");
}

GeneratePlaneMeshDialog::~GeneratePlaneMeshDialog() { delete ui_; }

double GeneratePlaneMeshDialog::getPlaneLength() const { return ui_->length_dist_spinbox->value(); }

double GeneratePlaneMeshDialog::getPlaneWidth() const { return ui_->width_dist_spinbox->value(); }

}  // namespace noether
