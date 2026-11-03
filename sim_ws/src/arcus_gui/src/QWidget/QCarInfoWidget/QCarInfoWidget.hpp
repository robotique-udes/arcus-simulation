#ifndef Q_CARINFOWIDGET_HPP
#define Q_CARINFOWIDGET_HPP

#include "UI_CarInfo.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>
#include <string>

class QCarInfoWidget : public QWidget  
{
    Q_OBJECT

    static constexpr const char* TOPIC_ODOM = "/odometry/filtered";
    static constexpr const char* DRIVE_TOPIC = "/drive";
    static constexpr const char* RISK_TOPIC = "/pure_pursuit/trajectory_risk";
    static constexpr const char* SOCKET_STATUS_TOPIC = "/socket_adapter/status";
    static constexpr const float pi = 3.14159265359;


  public:

    QCarInfoWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void updateSpeedIU(float speed_);
    void updateAngleIU(float angle_);
    void updateRiskIU(float risk_);
    void updateConnectionIU(bool connected_, QStringList topics_);
    
  private slots:
    void onUpdateAngleUI(float angle_);
    void onUpdateSpeedUI(float speed_);
    void onUpdateRiskUI(float risk_);
    void onUpdateConnectionUI(bool connected_, QStringList topics_);

  private:

    void setupUI(void);

    void initSubscriber(void);

    void CB_odom(nav_msgs::msg::Odometry& msg_);

    void CB_driver(ackermann_msgs::msg::AckermannDriveStamped& msg_);

    void CB_risk(std_msgs::msg::Float32& msg_);
    void CB_socketStatus(std_msgs::msg::String& msg_);

    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_odom;

    std::shared_ptr<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>> _sub_acker;

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float32>> _sub_risk;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> _sub_socket_status;

    std::shared_ptr<rclcpp::Node> _node;

    Ui::CarInfoWidget _ui;
};

#endif  // Q_CARINFO_WIDGET_HPP
