#include "QCarInfoWidget.hpp"

QCarInfoWidget::QCarInfoWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _ui.setupUi(this);

    this->initSubscriber();

    connect(this, &QCarInfoWidget::updateSpeedIU, this, &QCarInfoWidget::onUpdateSpeedUI);

    connect( this, &QCarInfoWidget::updateAngleIU, this, &QCarInfoWidget::onUpdateAngleUI);

    connect( this, &QCarInfoWidget::updateRiskIU, this, &QCarInfoWidget::onUpdateRiskUI);
}

void QCarInfoWidget::initSubscriber(void)
{
    if(_node)
    {
        _sub_odom = _node->create_subscription<nav_msgs::msg::Odometry>(TOPIC_ODOM,
                                                                        1,
                                                                        [this](nav_msgs::msg::Odometry msg_)
                                                                        {
                                                                            this->CB_odom(msg_);
                                                                        });

        _sub_acker = _node->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(DRIVE_TOPIC, 1, [this](ackermann_msgs::msg::AckermannDriveStamped msg_){
            this->CB_driver(msg_);
        });  
        
        _sub_risk = _node->create_subscription<std_msgs::msg::Float32>(RISK_TOPIC, 1, [this](std_msgs::msg::Float32 msg_){
            this->CB_risk(msg_);
        });
    } 
    else
    {
        assert(false && "Error: GUI node is null");
    }                                                                
    }

void QCarInfoWidget::CB_odom(nav_msgs::msg::Odometry& msg_)
{
    float speed = msg_.twist.twist.linear.x;

    emit this->updateSpeedIU(speed);
}

void QCarInfoWidget::CB_driver(ackermann_msgs::msg::AckermannDriveStamped& msg_)
{
    float angle = msg_.drive.steering_angle;

    emit this->updateAngleIU(angle);
}

void QCarInfoWidget::CB_risk(std_msgs::msg::Float32& msg_)
{
    float risk = msg_.data;

    emit this->updateRiskIU(risk);
}

void QCarInfoWidget::onUpdateSpeedUI(float speed_)
{
    this->_ui.lcdSpeed->display(QString::number(speed_,'f',2));
}

void QCarInfoWidget::onUpdateAngleUI(float angle_)
{
    this->_ui.lcdAngle->display(QString::number((180.0*angle_/pi),'f',2));
}

void QCarInfoWidget::onUpdateRiskUI(float risk_)
{
    this->_ui.lcdX1->display(QString::number(risk_,'f',2));
}

