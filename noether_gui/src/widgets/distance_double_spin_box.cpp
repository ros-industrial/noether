#include <noether_gui/widgets/distance_double_spin_box.h>

#include <QTextStream>
#include <QValidator>
#include <regex>

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
DistanceDoubleSpinBox::DistanceDoubleSpinBox(QWidget* parent) : QDoubleSpinBox(parent)
{
  setRange(-100.0, 100.0);
  setSingleStep(0.010);
}

QString DistanceDoubleSpinBox::textFromValue(double value_m) const
{
  double value;

  if (unit_ == "m")
    value = value_m;
  else if (unit_.empty())
    value = value_m;
  else if (unit_ == "cm")
    value = value_m * 100.0;
  else if (unit_ == "mm")
    value = value_m * 1000.0;
  else if (unit_ == "in" || unit_ == "\"")
    value = value_m / 0.0254;
  else if (unit_ == "ft" || unit_ == "\'")
    value = value_m / (0.0254 * 12.0);
  else if (unit_ == "yd")
    value = value_m / (0.0254 * 36.0);
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

double DistanceDoubleSpinBox::valueFromText(const QString& text) const
{
  double value;
  std::string unit;
  std::tie(value, unit) = split(text.toStdString());

  // If there is no unit defined, assume the last known unit is still active for the sake of conversion
  if (unit.empty())
    unit = unit_;

  // Convert the value to meters for internal storage
  double value_m;
  if (unit == "m")
    value_m = value;
  else if (unit == "cm")
    value_m = value / 100.0;
  else if (unit == "mm")
    value_m = value / 1000.0;
  else if (unit == "in" || unit == "\"")
    value_m = value * 0.0254;
  else if (unit == "ft" || unit == "\'")
    value_m = value * 0.0254 * 12.0;
  else if (unit == "yd")
    value_m = value * 0.0254 * 36.0;
  else
    return value;

  // Update the unit
  unit_ = unit;

  return value_m;
}

QValidator::State DistanceDoubleSpinBox::validate(QString& input, int& pos) const { return QValidator::Acceptable; }

}  // namespace noether
