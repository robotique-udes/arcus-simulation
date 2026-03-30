#include "QSpeedWidget.hpp"

QSpeedWidget::QSpeedWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
    //_odomTimeout(rclcpp::Duration::from_seconds(WATCH_DOG_TIMEOUT_S))
{
    _ui.setupUi(this);

    //_lastOdomMsgTime = _node->now();

    //this->setupUI();
    this->initOdomSubscriber();

    connect(this, &QSpeedWidget::updateSpeedIU, this, &QSpeedWidget::onUpdateSpeedUI);
}

/*void QSpeedWidget::setupUI(void)
{   

    _ui.SpeedLCD->setText(" --.-- ");
}*/

void QSpeedWidget::initOdomSubscriber(void)
{
    if (_node)
    {
        _sub_odom = _node->create_subscription<nav_msgs::msg::Odometry>(TOPIC_ODOM,
                                                                        1,
                                                                        [this](nav_msgs::msg::Odometry msg_)
                                                                        {
                                                                            this->CB_odom(msg_);
                                                                        });
    }
    else
    {
        assert(false && "Error: GUI node is null");
    }
}

void QSpeedWidget::CB_odom(nav_msgs::msg::Odometry& msg_)
{
    float speed = msg_.twist.twist.linear.x;

    emit this->updateSpeedIU(speed);
}

void QSpeedWidget::DisplayLCDUI(float speed_)
{
    _ui.SpeedLCD_3->display(QString::number(speed_,'f',2));
}

void QSpeedWidget::onUpdateSpeedUI(float speed_)
{
    this->DisplayLCDUI(speed_);
}