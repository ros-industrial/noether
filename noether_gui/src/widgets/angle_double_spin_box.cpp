#include <noether_gui/widgets/angle_double_spin_box.h>

#include <QTextStream>
#include <QValidator>
#include <regex>
#include <cmath>

/**
 * @brief Helper function for splitting text into a value and unit
 */
static std::tuple<double, std::string> split(const std::string& text)
{
  const std::regex re("^([\\d\\.]+)\\s*(\\S*)$");
  std::smatch matches;

  if (!std::regex_search(text, matches, re))
    return std::make_tuple(0.0, "");

  return std::make_tuple(std::stod(matches.str(1)), matches.str(2));
}

namespace noether
{
AngleDoubleSpinBox::AngleDoubleSpinBox(QWidget* parent) : QDoubleSpinBox(parent)
{
  setRange(-M_PI, M_PI);
  setSingleStep(1.0 * M_PI / 180.0);
}

QString AngleDoubleSpinBox::textFromValue(double value_rad) const
{
  double value;

  if (unit_ == "rad")
    value = value_rad;
  else if (unit_.empty())
    value = value_rad;
  else if (unit_ == "deg")
    value = value_rad * 180.0 / M_PI;
  else
    throw std::runtime_error("Invalid internal unit");

  QString text;
  QTextStream stream(&text);
  stream.setRealNumberPrecision(decimals());
  stream << value;
  if (!unit_.empty())
    stream << " " << QString::fromStdString(unit_);

  return text;
}

double AngleDoubleSpinBox::valueFromText(const QString& text) const
{
  double value;
  std::string unit;
  std::tie(value, unit) = split(text.toStdString());

  // If there is no unit defined, assume the last known unit is still active for the sake of conversion
  if (unit.empty())
    unit = unit_;

  // Convert the value to meters for internal storage
  double value_rad;
  if (unit == "rad")
    value_rad = value;
  else if (unit == "deg")
    value_rad = value * M_PI / 180.0;
  else
    return value;

  // Update the unit
  unit_ = unit;

  return value_rad;
}

QValidator::State AngleDoubleSpinBox::validate(QString& input, int& pos) const { return QValidator::Acceptable; }

}  // namespace noether
