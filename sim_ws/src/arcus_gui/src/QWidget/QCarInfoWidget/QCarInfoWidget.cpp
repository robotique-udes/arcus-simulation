#include "QCarInfoWidget.hpp"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>

QCarInfoWidget::QCarInfoWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _ui.setupUi(this);

    this->initSubscriber();

    connect(this, &QCarInfoWidget::updateSpeedIU, this, &QCarInfoWidget::onUpdateSpeedUI);

    connect( this, &QCarInfoWidget::updateAngleIU, this, &QCarInfoWidget::onUpdateAngleUI);

    connect( this, &QCarInfoWidget::updateRiskIU, this, &QCarInfoWidget::onUpdateRiskUI);
    connect(this, &QCarInfoWidget::updateConnectionIU, this, &QCarInfoWidget::onUpdateConnectionUI);
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

        _sub_socket_status = _node->create_subscription<std_msgs::msg::String>(
            SOCKET_STATUS_TOPIC,
            rclcpp::QoS(1).transient_local(),
            [this](std_msgs::msg::String msg_){
            this->CB_socketStatus(msg_);
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

void QCarInfoWidget::CB_socketStatus(std_msgs::msg::String& msg_)
{
    const QJsonDocument document = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg_.data));
    if (!document.isObject())
    {
        return;
    }

    const QJsonObject object = document.object();
    QStringList topics;
    for (const QJsonValue& value : object.value("topics").toArray())
    {
        const QJsonObject topic = value.toObject();
        const QString name = topic.value("name").toString();
        const QString type = topic.value("type").toString();
        if (!name.isEmpty())
        {
            topics.append(type.isEmpty() ? name : name + " (" + type + ")");
        }
    }

    emit updateConnectionIU(object.value("connected").toBool(), topics);
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

void QCarInfoWidget::onUpdateConnectionUI(bool connected_, QStringList topics_)
{
    _ui.connectionIndicator->setText(
        connected_ ? QStringLiteral("Remote: connected")
                   : QStringLiteral("Remote: disconnected"));
    _ui.connectionIndicator->setStyleSheet(
        connected_ ? QStringLiteral("color: #6fdc6f;")
                   : QStringLiteral("color: #ff6b6b;"));
    _ui.remoteTopics->clear();
    _ui.remoteTopics->addItems(topics_);
    _ui.topicCount->setText(
        QStringLiteral("Topics: %1").arg(topics_.size()));
}
