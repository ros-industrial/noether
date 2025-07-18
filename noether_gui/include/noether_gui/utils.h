#pragma once

#include <QLayoutItem>
#include <QLayout>
#include <QStringList>
#include <QWidget>
#include <string>
#include <vector>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/exceptions.h>

namespace noether
{
inline QStringList toQStringList(const std::vector<std::string>& list)
{
  QStringList out;
  out.reserve(list.size());
  std::transform(list.begin(), list.end(), std::back_inserter(out), &QString::fromStdString);

  // Insert blank at the beginning
  out.insert(out.begin(), QString{});

  return out;
}

/**
 * @brief Overwrites one widget within a layout with another widget
 */
inline void overwriteWidget(QLayout* layout, QWidget*& from, QWidget* to)
{
  if (!from)
    throw std::runtime_error("From widget is a nullptr");
  if (!to)
    throw std::runtime_error("To widget is a nullptr");
  if (!layout)
    throw std::runtime_error("Layout is a nullptr");

  QLayoutItem* ret = layout->replaceWidget(from, to);
  if (!ret)
    throw std::runtime_error("Widget replacement returned a nullptr");
  if (ret->widget() != from)
    throw std::runtime_error("Widget replacement did not return the correct replaced widget");

  // Delete the replaced widget (which is no longer managed), and reset it to the widget that replaced it for future
  // reference
  delete from;
  from = to;
}

/**
 * @details Adapted from https://en.cppreference.com/w/cpp/error/throw_with_nested
 */
inline void printException(const std::exception& e, std::ostream& ss, int level = 0)
{
  ss << std::string(level * 4, ' ') << e.what() << '\n';
  try
  {
    std::rethrow_if_nested(e);
  }
  catch (const std::exception& nested_exception)
  {
    printException(nested_exception, ss, level + 1);
  }
}

}  // namespace noether
