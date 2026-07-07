#include "QParamSaverWidget.hpp"

QParamSaverWidget::QParamSaverWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _ui.setupUi(this);

    _paramSaverClient = _node->create_client<std_srvs::srv::Trigger>("/arcus/save_parameters");

    this->connectSignals();
}

void QParamSaverWidget::connectSignals(void)
{
    connect(_ui.globalSaveButton, &QPushButton::clicked, this, &QParamSaverWidget::onGlobalSaveClicked);
}

void QParamSaverWidget::onGlobalSaveClicked(void)
{
    if (!_paramSaverClient->wait_for_service(std::chrono::milliseconds(500)))
    {
        RCLCPP_ERROR(_node->get_logger(), "Global parameter saver node is not running!");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    RCLCPP_INFO(_node->get_logger(), "Sending call to save all active parameters to disk...");
    
    _paramSaverClient->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(_node->get_logger(), "SUCCESS: %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(_node->get_logger(), "FAILURE: %s", response->message.c_str());
            }
        });
}