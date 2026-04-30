#ifndef Q_SAFETY_WIDGET_HPP
#define Q_SAFETY_WIDGET_HPP

#include "UI_SafetyWidget.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>
#include "std_msgs/msg/bool.hpp"

class QSafetyWidget : public QWidget
{
    Q_OBJECT

    static constexpr const char* DEADMAN_TOPIC = "/deadman_active";

  public:
    QSafetyWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void updateDeadmanSwitchUI(bool isActive_);
  private slots:
    void OnupdateDeadmanSwitchUI(bool isActive_);

  private:
    void setupUI(void);
    void initRosElements(void);

    void CB_deadman(const std_msgs::msg::Bool& msg_);

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> _sub_deadman;

    std::shared_ptr<rclcpp::Node> _node;
    Ui::SafetyWidget _ui;
};

#endif  // Q_SAFETY_WIDGET_HPP
