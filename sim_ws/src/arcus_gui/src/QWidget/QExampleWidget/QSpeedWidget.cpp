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

        /*_timer_odomPub = _node->create_wall_timer(std::chrono::milliseconds(DELAY_CHECK_ODOM_MS),
                                                  [this](void)
                                                  {
                                                      this->CB_odomPubCount();
                                                  });

        _watchdog_odom = _node->create_wall_timer(std::chrono::milliseconds(WATCH_DOG_DELAY_MS),
                                                  [this](void)
                                                  {
                                                      this->CB_odomTimeout();
                                                  });*/
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

/*void QSpeedWidget::CB_odomPubCount()
{
    _lastOdomMsgTime = _node->now();
    size_t count = _node->count_publishers(TOPIC_ODOM);
    if (!count)
    {
        _ui.title->setText("No Speed Received!");
    }
}*/

/*void QSpeedWidget::CB_odomTimeout()
{
    if ((_node->now() - _lastOdomMsgTime) > _odomTimeout)
    {
        _ui.title->setText("No Speed Received!");
    }
}*/
