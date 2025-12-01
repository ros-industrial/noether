#pragma once

#include <QDoubleSpinBox>

namespace noether
{
/**
 * @brief Double spin box widget class that supports distance units
 * @details Supported units are meters (m), centimeters (cm), millimeters (mm), inches (in, "), feet (ft, '), and yards
 * (yd)
 */
class DistanceDoubleSpinBox : public QDoubleSpinBox
{
public:
  DistanceDoubleSpinBox(QWidget* parent = nullptr);

  double valueFromText(const QString& text) const override;
  QString textFromValue(double val) const override;
  QValidator::State validate(QString& input, int& pos) const override;

protected:
  mutable std::string unit_ = "m";
};

}  // namespace noether
