#ifndef Q_ERROR_CODE
#define Q_ERROR_CODE

#include "UI_ErrorCode.h"
#include "nav_msgs/msg/odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QErrocCode : public QWidget
{
    Q_OBJECT

    static constexpr const char* TOPIC_ODOM = "/node_error_code";

    static constexpr const size_t DELAY_CHECK_ERROR_MS = 10UL;

  public:
    QExampleWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void updateErrorDisplay(Erro);

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

#endif  // Q_EXAMPLE_WIDGET_HPP
