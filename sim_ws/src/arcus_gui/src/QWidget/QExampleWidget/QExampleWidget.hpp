#ifndef Q_EXAMPLE_WIDGET_HPP
#define Q_EXAMPLE_WIDGET_HPP

#include "UI_ExampleWidget.h"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QExampleWidget : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_ODOM = "/ego_racecar/odom";

    static constexpr const size_t DELAY_CHECK_ODOM_MS = 100UL;
    static constexpr const size_t WATCH_DOG_DELAY_MS = 1000UL;
    static constexpr const double WATCH_DOG_TIMEOUT_S = 1.0F;

  public:
    QExampleWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void updateXYZPositionIU(float x_, float y_, float z_);

  private slots:
    void onUpdateXYZPositionUI(float x_, float y_, float z_);

  private:
    void setupUI(void);

    void initOdomSubscriber(void);

    void CB_odom(nav_msgs::msg::Odometry& msg_);

    void CB_odomPubCount();

    void CB_odomTimeout();

    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_odom;

    rclcpp::TimerBase::SharedPtr _timer_odomPub;

    rclcpp::TimerBase::SharedPtr _watchdog_odom;

    rclcpp::Time _lastOdomMsgTime;

    std::shared_ptr<rclcpp::Node> _node;

    rclcpp::Duration _odomTimeout;

    Ui::ExampleWidget _ui;
};

#endif  // QTOP_UTILITY_BAR