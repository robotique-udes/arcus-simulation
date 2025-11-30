#include "QExampleWidget.hpp"

QExampleWidget::QExampleWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _odomTimeout(rclcpp::Duration::from_seconds(WATCH_DOG_TIMEOUT_S))
{
    _ui.setupUi(this);

    _lastOdomMsgTime = _node->now();

    this->setupUI();
    this->initOdomSubscriber();

    connect(this, &QExampleWidget::updateXYZPositionIU, this, &QExampleWidget::onUpdateXYZPositionUI);
}

void QExampleWidget::setupUI(void)
{
    _ui.title->setText("Car Position: ");
    _ui.posXText->setText("X : --.-- ");
    _ui.posYText->setText("Car --.--: ");
    _ui.posZText->setText("Car --.--: ");
}

void QExampleWidget::initOdomSubscriber(void)
{
    if (_node)
    {
        _sub_odom = _node->create_subscription<nav_msgs::msg::Odometry>(TOPIC_ODOM,
                                                                        1,
                                                                        [this](nav_msgs::msg::Odometry msg_)
                                                                        {
                                                                            this->CB_odom(msg_);
                                                                        });

        _timer_odomPub = _node->create_wall_timer(std::chrono::milliseconds(DELAY_CHECK_ODOM_MS),
                                                  [this](void)
                                                  {
                                                      this->CB_odomPubCount();
                                                  });

        _watchdog_odom = _node->create_wall_timer(std::chrono::milliseconds(WATCH_DOG_DELAY_MS),
                                                  [this](void)
                                                  {
                                                      this->CB_odomTimeout();
                                                  });
    }
    else
    {
        assert(false && "Error: GUI node is null");
    }
}

void QExampleWidget::CB_odom(nav_msgs::msg::Odometry& msg_)
{
    float x = msg_.pose.pose.position.x;
    float y = msg_.pose.pose.position.y;
    float z = msg_.pose.pose.position.z;

    emit this->updateXYZPositionIU(x, y, z);
}

void QExampleWidget::onUpdateXYZPositionUI(float x_, float y_, float z_)
{
    _ui.title->setText("Car Position: ");
    _ui.posXText->setText("X: " + QString::number(x_) + " m");
    _ui.posYText->setText("Y: " + QString::number(y_) + " m");
    _ui.posZText->setText("Z: " + QString::number(z_) + " m");
}

void QExampleWidget::CB_odomPubCount()
{
    _lastOdomMsgTime = _node->now();
    size_t count = _node->count_publishers(TOPIC_ODOM);
    if (!count)
    {
        _ui.title->setText("Not Posistion Received!");
    }
}

void QExampleWidget::CB_odomTimeout()
{
    if ((_node->now() - _lastOdomMsgTime) > _odomTimeout)
    {
        _ui.title->setText("Not Posistion Received!");
    }
}
