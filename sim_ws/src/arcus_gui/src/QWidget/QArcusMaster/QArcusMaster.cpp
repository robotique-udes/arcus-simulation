#include "QArcusMaster.hpp"

#include <QtWidgets/QStyle>

QArcusMaster::QArcusMaster(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _safetyWidget(node_, parent_),
    _node(node_),
    _masterNodeTimeout(std::chrono::milliseconds(MASTER_NODE_TIMEOUT_MS)),
    _lastMasterErrorMsg(_node->get_clock()->now())
{
    this->setupUI();
    this->initRosElements();
    connect(this, &QArcusMaster::displayError, this, &QArcusMaster::onDisplayError);
    connect(this, &QArcusMaster::displayConnection, this, &QArcusMaster::onDisplayConnection);
}
    
void QArcusMaster::onDisplayError(uint16_t errorCode_){
    if(errorCode_ == arcus_msgs::msg::ErrorCode::OK)
    {
        _ui.errorData->setText("OK");
        _ui.errorData->setProperty("state", "success"); 
        _ui.errorData->style()->unpolish(_ui.errorData);
        _ui.errorData->style()->polish(_ui.errorData);
    }
    else
    {
        switch(errorCode_)
        {
            case arcus_msgs::msg::ErrorCode::OFFLINE:
                _ui.errorData->setText("OFFLINE ERROR");
                break;
            case arcus_msgs::msg::ErrorCode::TIMEOUT:
                _ui.errorData->setText("TIMEOUT ERROR");
                break;
            case arcus_msgs::msg::ErrorCode::INVALID_DATA:
                _ui.errorData->setText("INVALID DATA");
                break;
            case arcus_msgs::msg::ErrorCode::EMERGENCY_BRAKE:
                _ui.errorData->setText("EMERGENCY BRAKE");
                break;
            default:
                _ui.errorData->setText("UNKNOWN ERROR CODE");
                break;
        }

        _ui.errorData->setProperty("state", "error"); 
        _ui.errorData->style()->unpolish(_ui.errorData);
        _ui.errorData->style()->polish(_ui.errorData);
    }
}

void QArcusMaster::onDisplayConnection(bool isConnected)
{
    if(isConnected)
    {
        _ui.statusData->setText("CONNECTED");
        _ui.statusData->setProperty("state", "success"); 
        _ui.statusData->style()->unpolish(_ui.statusData);
        _ui.statusData->style()->polish(_ui.statusData);
    }
    else
    {
        _ui.statusData->setText("DISCONNECTED");
        _ui.statusData->setProperty("state", "error"); 
        _ui.statusData->style()->unpolish(_ui.statusData);
        _ui.statusData->style()->polish(_ui.statusData);
    }
}

void QArcusMaster::setupUI(void)
{
    _ui.setupUi(this);

    _ui.gridLayout->addWidget(&_safetyWidget);
}

void QArcusMaster::initRosElements(void)
{
    if (_node)
        {
            _sub_errorCode = _node->create_subscription<arcus_msgs::msg::ErrorCode>(ERROR_CODE_TOPIC,
                                                                            1,
                                                                            [this](const arcus_msgs::msg::ErrorCode & msg_)
                                                                            {
                                                                                this->CB_errorCode(msg_);
                                                                            });

            _sub_heartbeatMaster = _node->create_subscription<std_msgs::msg::Bool>(HEARTBEAT_MASTER_TOPIC,
                                                                            1,
                                                                            [this](const std_msgs::msg::Bool & msg_)
                                                                            {
                                                                                this->CB_receiveHearbeat(msg_);
                                                                            });

            _watchdogMasterNode = _node->create_wall_timer(std::chrono::milliseconds(WATCHDOG_CHECK_MS),
                                                    [this](void)
                                                    {
                                                        this->CB_heartbeatMaster();
                                                    });
        }
        else
        {
            assert(false && "Error: GUI node is null");
        }
}

void QArcusMaster::CB_errorCode(const arcus_msgs::msg::ErrorCode& msg_)
{
    emit displayError(msg_.error_code);
}

void QArcusMaster::CB_receiveHearbeat(const std_msgs::msg::Bool& msg_)
{
    if (msg_.data)
    {
        _lastMasterErrorMsg = _node->get_clock()->now();
    }
}

void QArcusMaster::CB_heartbeatMaster()
{
    bool currentlyConnected = (_node->get_clock()->now() - _lastMasterErrorMsg) <= _masterNodeTimeout;
    
    if (currentlyConnected != _isConnected)
    {
        _isConnected = currentlyConnected;
        emit displayConnection(_isConnected);
    }
}

