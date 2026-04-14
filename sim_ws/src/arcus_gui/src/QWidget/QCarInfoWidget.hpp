#ifndef Q_CARINFOWIDGET_HPP
#define Q_CARINFOWIDGET_HPP

#include "UI_CarInfo.h"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QCarInfoWidget : public QWidget  
{
    Q_OBJECT

    static constexpr const char* TOPIC_ODOM = "/odometry/filtered";
    static constexpr const char* DRIVE_TOPIC = "/drive";


  public:

    QCarInfoWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void updateSpeedIU(float speed_);
    void updateAngleIU(float angle_);

  private slots:
    void onUpdateSpeedUI(float speed_);
    void onUpdateAngleUI(float angle_);

  private:

    float pi = 3.14159265359;

    void setupUI(void);

    void DisplayLCDSPEEDUI(float speed_);

    void DisplayLCDANGLEUI(float angle_);

    void initOdomSubscriber(void);

    void initDriveSubscriber(void);

    void CB_odom(nav_msgs::msg::Odometry& msg_);

    void CB_driver(ackermann_msgs::msg::AckermannDriveStamped& msg_);

    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> _sub_odom;

    std::shared_ptr<rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>> _sub_acker;

    std::shared_ptr<rclcpp::Node> _node;

    Ui::CarInfoWidget _ui;
};

#endif  // Q_CARINFO_WIDGET_HPP
