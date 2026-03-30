#ifndef Q_SPEEDWIDGET_HPP
#define Q_SPEEDWIDGET_HPP

#include "UI_SpeedWidget.h"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QSpeedWidget : public QWidget  
{
    Q_OBJECT

    static constexpr const char* TOPIC_ODOM = "/odometry/filtered";

  public:
    QSpeedWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void updateSpeedIU(float speed_);

  private slots:
    void onUpdateSpeedUI(float speed_);

  private:
    void setupUI(void);
    void DisplayLCDUI(float speed_);

    void initOdomSubscriber(void);

    void CB_odom(nav_msgs::msg::Odometry& msg_);

    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_odom;

    std::shared_ptr<rclcpp::Node> _node;

    Ui::Speed _ui;
};

#endif  // Q_SPEED_WIDGET_HPP
