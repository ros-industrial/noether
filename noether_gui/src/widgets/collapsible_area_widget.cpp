#include <noether_gui/widgets/collapsible_area_widget.h>

#include <QParallelAnimationGroup>
#include <QPropertyAnimation>
#include <QGridLayout>
#include <QToolButton>
#include <QFrame>
#include <QScrollArea>

static const int ANIMATION_DURATION = 100;  // milliseconds

namespace noether
{
CollapsibleArea::CollapsibleArea(const QString& label, QWidget* parent)
  : QWidget(parent)
  , label_(label)
  , layout(new QGridLayout(this))
  , tool_button(new QToolButton(this))
  , line(new QFrame(this))
  , content(new QScrollArea(this))
  , animation_(new QParallelAnimationGroup(this))
{
  tool_button->setStyleSheet("QToolButton { border: none; }");
  tool_button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  tool_button->setArrowType(Qt::ArrowType::RightArrow);
  tool_button->setCheckable(true);
  tool_button->setChecked(false);
  tool_button->setText(label.isEmpty() ? "..." : label);

  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);
  line->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);

  // don't waste space
  layout->setVerticalSpacing(0);
  layout->setContentsMargins(0, 0, 0, 0);
  int row = 0;
  layout->addWidget(tool_button, row, 0, 1, 1, Qt::AlignLeft);
  layout->addWidget(line, row++, 2, 1, 1);
  layout->addWidget(content, row, 0, 1, 3);
  setLayout(layout);

  // Start minimized
  content->setStyleSheet("QScrollArea { border: none; }");
  content->setMaximumHeight(0);
  content->setMinimumHeight(0);

  animation_->addAnimation(new QPropertyAnimation(this, "minimumHeight"));
  animation_->addAnimation(new QPropertyAnimation(this, "maximumHeight"));
  animation_->addAnimation(new QPropertyAnimation(content, "maximumHeight"));

  connect(tool_button, &QToolButton::clicked, [this](const bool checked) {
    tool_button->setArrowType(checked ? Qt::ArrowType::DownArrow : Qt::ArrowType::RightArrow);
    animation_->setDirection(checked ? QAbstractAnimation::Forward : QAbstractAnimation::Backward);
    animation_->start();
  });
}

CollapsibleArea::~CollapsibleArea()
{
  delete layout;
  delete tool_button;
  delete line;
  delete content;
  delete animation_;
}

CollapsibleArea::CollapsibleArea(QWidget* parent) : CollapsibleArea(QString(), parent) {}

void CollapsibleArea::setWidget(QWidget* widget)
{
  // Replace the content widget
  QWidget* old_widget = content->takeWidget();
  delete old_widget;
  content->setWidget(widget);

  const auto collapsed_height = sizeHint().height() - content->maximumHeight();
  const auto content_height = std::max(0, widget->sizeHint().height());

  for (int i = 0; i < animation_->animationCount() - 1; ++i)
  {
    QPropertyAnimation* prop_animation = static_cast<QPropertyAnimation*>(animation_->animationAt(i));
    prop_animation->setDuration(ANIMATION_DURATION);
    prop_animation->setStartValue(collapsed_height);
    prop_animation->setEndValue(collapsed_height + content_height);
  }

  QPropertyAnimation* content_animation =
      static_cast<QPropertyAnimation*>(animation_->animationAt(animation_->animationCount() - 1));
  content_animation->setDuration(ANIMATION_DURATION);
  content_animation->setStartValue(0);
  content_animation->setEndValue(content_height);
}

QWidget* CollapsibleArea::getWidget() const { return content->widget(); }

QString CollapsibleArea::getLabel() const { return label_; }

}  // namespace noether
