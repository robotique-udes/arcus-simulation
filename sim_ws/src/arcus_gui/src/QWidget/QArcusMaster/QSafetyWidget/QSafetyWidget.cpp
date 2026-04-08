#include "QSafetyWidget.hpp"

#include <QtWidgets/QStyle>

QSafetyWidget::QSafetyWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    this->setupUI();
    this->initRosElements();

    connect(this, &QSafetyWidget::updateDeadmanSwitchUI, this, &QSafetyWidget::OnupdateDeadmanSwitchUI);
}

void QSafetyWidget::setupUI(void)
{   
    _ui.setupUi(this);
}

void QSafetyWidget::OnupdateDeadmanSwitchUI(bool isActive_)
{
    if(isActive_)
    {
        _ui.deadmanData->setText("ACTIVE");
        _ui.deadmanData->setProperty("state", "success"); 
        _ui.deadmanData->style()->unpolish(_ui.deadmanData);
        _ui.deadmanData->style()->polish(_ui.deadmanData);
    }
    else
    {
        _ui.deadmanData->setText("INACTIVE");
        _ui.deadmanData->setProperty("state", "error"); 
        _ui.deadmanData->style()->unpolish(_ui.deadmanData);
        _ui.deadmanData->style()->polish(_ui.deadmanData);
    }
}

void QSafetyWidget::initRosElements(void)
{
    if (_node)
        {
            _sub_deadman = _node->create_subscription<std_msgs::msg::Bool>(DEADMAN_TOPIC,
                                                                            1,
                                                                            [this](const std_msgs::msg::Bool & msg_)
                                                                            {
                                                                                this->CB_deadman(msg_);
                                                                            });
        }
        else
        {
            assert(false && "Error: GUI node is null");
        }
}

void QSafetyWidget::CB_deadman(const std_msgs::msg::Bool& msg_)
{
    emit updateDeadmanSwitchUI(msg_.data);
}