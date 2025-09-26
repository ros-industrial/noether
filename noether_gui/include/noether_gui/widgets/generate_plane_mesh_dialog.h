#pragma once

#include <QDialog>

namespace Ui
{
class GeneratePlaneMeshDialog;
}

namespace noether
{
class GeneratePlaneMeshDialog : public QDialog
{
public:
  explicit GeneratePlaneMeshDialog(QWidget* parent = nullptr);
  ~GeneratePlaneMeshDialog();

  double getPlaneLength() const;
  double getPlaneWidth() const;

private:
  Ui::GeneratePlaneMeshDialog* ui_;
};
}  // namespace noether
