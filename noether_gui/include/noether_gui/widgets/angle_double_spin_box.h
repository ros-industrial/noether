#pragma once

#include <QDoubleSpinBox>

namespace noether
{
/**
 * @brief Double spin box widget class that supports angle units
 * @details Supported units are degrees (deg) and radians (rad)
 */
class AngleDoubleSpinBox : public QDoubleSpinBox
{
public:
  AngleDoubleSpinBox(QWidget* parent = nullptr);

  double valueFromText(const QString& text) const override;
  QString textFromValue(double val) const override;
  QValidator::State validate(QString& input, int& pos) const override;

protected:
  mutable std::string unit_ = "deg";
};

}  // namespace noether
