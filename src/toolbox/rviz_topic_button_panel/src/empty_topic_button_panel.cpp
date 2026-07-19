#include "rviz_topic_button_panel/empty_topic_button_panel.hpp"

#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace rviz_topic_button_panel
{

EmptyTopicButtonPanel::EmptyTopicButtonPanel(QWidget * parent)
: rviz_common::Panel(parent),
  description_label_(new QLabel()),
  first_button_(new QPushButton()),
  second_button_(new QPushButton()),
  description_("Publish an action:"),
  first_label_("Action 1"),
  first_topic_("/rviz/action_1"),
  second_label_("Action 2"),
  second_topic_("/rviz/action_2")
{
  auto * layout = new QVBoxLayout(this);
  layout->addWidget(description_label_);
  first_button_->setMinimumHeight(44);
  layout->addWidget(first_button_);
  second_button_->setMinimumHeight(44);
  layout->addWidget(second_button_);
  layout->addStretch();
  connect(first_button_, &QPushButton::clicked, this, &EmptyTopicButtonPanel::publishFirst);
  connect(second_button_, &QPushButton::clicked, this, &EmptyTopicButtonPanel::publishSecond);
  updateWidgets();
}

void EmptyTopicButtonPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  createPublishers();
}

void EmptyTopicButtonPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString value;
  if (config.mapGetString("Description", &value)) {
    description_ = value.toStdString();
  }
  if (config.mapGetString("Button 1 Label", &value)) {
    first_label_ = value.toStdString();
  }
  if (config.mapGetString("Button 1 Topic", &value)) {
    first_topic_ = value.toStdString();
  }
  if (config.mapGetString("Button 2 Label", &value)) {
    second_label_ = value.toStdString();
  }
  if (config.mapGetString("Button 2 Topic", &value)) {
    second_topic_ = value.toStdString();
  }
  updateWidgets();
  createPublishers();
}

void EmptyTopicButtonPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Description", QString::fromStdString(description_));
  config.mapSetValue("Button 1 Label", QString::fromStdString(first_label_));
  config.mapSetValue("Button 1 Topic", QString::fromStdString(first_topic_));
  config.mapSetValue("Button 2 Label", QString::fromStdString(second_label_));
  config.mapSetValue("Button 2 Topic", QString::fromStdString(second_topic_));
}

void EmptyTopicButtonPanel::createPublishers()
{
  if (!node_) {
    return;
  }
  first_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(first_topic_, 1);
  second_publisher_ = node_->create_publisher<std_msgs::msg::Empty>(second_topic_, 1);
}

void EmptyTopicButtonPanel::updateWidgets()
{
  description_label_->setText(QString::fromStdString(description_));
  first_button_->setText(QString::fromStdString(first_label_));
  second_button_->setText(QString::fromStdString(second_label_));
}

void EmptyTopicButtonPanel::publishFirst()
{
  if (first_publisher_) {
    first_publisher_->publish(std_msgs::msg::Empty());
  }
}

void EmptyTopicButtonPanel::publishSecond()
{
  if (second_publisher_) {
    second_publisher_->publish(std_msgs::msg::Empty());
  }
}

}  // namespace rviz_topic_button_panel

PLUGINLIB_EXPORT_CLASS(
  rviz_topic_button_panel::EmptyTopicButtonPanel,
  rviz_common::Panel)
