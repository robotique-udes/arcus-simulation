#include "QCarInfoWidget.hpp"

QCarInfoWidget::QCarInfoWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _ui.setupUi(this);

    this->initOdomSubscriber();

    connect(this, &QCarInfoWidget::updateSpeedIU, this, &QCarInfoWidget::onUpdateSpeedUI);
}

void QCarInfoWidget::initOdomSubscriber(void)
{
    if (_node)
    {
        _sub_odom = _node->create_subscription<nav_msgs::msg::Odometry>(TOPIC_ODOM,
                                                                        1,
                                                                        [this](nav_msgs::msg::Odometry msg_)
                                                                        {
                                                                            this->CB_odom(msg_);
                                                                        });

        _sub_angle = _node->create_subscription<nav_msgs::msg::Odometry
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
    _ui.lcdSpeed->display(QString::number(speed_,'f',2));
}

void QSpeedWidget::onUpdateSpeedUI(float speed_)
{
    this->DisplayLCDUI(speed_);
}


