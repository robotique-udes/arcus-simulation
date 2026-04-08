#ifndef Q_ARCUS_MASTER_HPP
#define Q_ARCUS_MASTER_HPP

#include "UI_ArcusMaster.h"
#include "QSafetyWidget/QSafetyWidget.hpp"
#include "arcus_msgs/msg/error_code.hpp"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QArcusMaster : public QWidget
{
    Q_OBJECT

    // Error code topic also used as heartbeat
    static constexpr const char* ERROR_CODE_TOPIC = "/master_error_code";
    static constexpr const uint32_t MASTER_NODE_TIMEOUT_MS = 100;
    static constexpr const uint32_t WATCHDOG_CHECK_MS = 33;

  public:
    QArcusMaster(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  signals:
    void displayError(uint16_t errorCode_);
    void displayConnection(bool isConnected);

  private slots:
    void onDisplayError(uint16_t errorCode);
    void onDisplayConnection(bool isConnected);

  private:
    void setupUI(void);

    void initRosElements(void);
    
    void CB_errorCode(const arcus_msgs::msg::ErrorCode& msg_);
    void CB_heartbeatMaster();

    QSafetyWidget _safetyWidget;

    bool _isConnected = false;

    std::shared_ptr<rclcpp::Subscription<arcus_msgs::msg::ErrorCode>> _sub_errorCode;

    rclcpp::TimerBase::SharedPtr _watchdogMasterNode;

    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Duration _masterNodeTimeout;
    rclcpp::Time _lastMasterErrorMsg;

    Ui::ArcusMaster _ui;
};

#endif  // Q_ARCUS_MASTER_HPP
