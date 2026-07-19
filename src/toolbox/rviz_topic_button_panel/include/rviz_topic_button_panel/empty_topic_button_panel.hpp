#ifndef RVIZ_TOPIC_BUTTON_PANEL__EMPTY_TOPIC_BUTTON_PANEL_HPP_
#define RVIZ_TOPIC_BUTTON_PANEL__EMPTY_TOPIC_BUTTON_PANEL_HPP_

#include <memory>
#include <string>

#include <QLabel>
#include <QPushButton>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/empty.hpp>

namespace rviz_topic_button_panel
{

class EmptyTopicButtonPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit EmptyTopicButtonPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void publishFirst();
  void publishSecond();

private:
  void createPublishers();
  void updateWidgets();

  QLabel * description_label_;
  QPushButton * first_button_;
  QPushButton * second_button_;
  std::string description_;
  std::string first_label_;
  std::string first_topic_;
  std::string second_label_;
  std::string second_topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr first_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr second_publisher_;
};

}  // namespace rviz_topic_button_panel

#endif  // RVIZ_TOPIC_BUTTON_PANEL__EMPTY_TOPIC_BUTTON_PANEL_HPP_
