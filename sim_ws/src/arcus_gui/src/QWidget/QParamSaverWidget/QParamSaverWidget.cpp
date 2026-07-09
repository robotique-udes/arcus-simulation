#include "QParamSaverWidget.hpp"

QParamSaverWidget::QParamSaverWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_)
{
    _ui.setupUi(this);

    _paramSaverClient = _node->create_client<std_srvs::srv::Trigger>("/arcus/save_parameters");
    _getParamClient = _node->create_client<rcl_interfaces::srv::GetParameters>("/param_saver_node/get_parameters");
    _setParamClient = _node->create_client<rcl_interfaces::srv::SetParameters>("/param_saver_node/set_parameters");

    this->connectSignals();
    this->reloadProfiles();
}

void QParamSaverWidget::connectSignals(void)
{
    connect(_ui.globalSaveButton, &QPushButton::clicked, this, &QParamSaverWidget::onGlobalSaveClicked);
    connect(_ui.refreshButton, &QPushButton::clicked, this, &QParamSaverWidget::reloadProfiles);
    connect(_ui.configProfilesDropdown, &QComboBox::currentTextChanged, this, &QParamSaverWidget::onProfileSwitch);
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

void QParamSaverWidget::reloadProfiles(void)
{
    if (!_getParamClient->wait_for_service(std::chrono::milliseconds(500)))
    {
        RCLCPP_ERROR(_node->get_logger(), "Parameter service on param_saver_node is not available!");
        return;
    }

    this->setUiEnabled(false);

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("available_profiles");

    _getParamClient->async_send_request(request,
        [this](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
            this->setUiEnabled(true);

            auto response = future.get();
            if (!response->values.empty() && response->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY) 
            {
                std::vector<std::string> configs = response->values[0].string_array_value;
                
                _isUpdatingDropdown = true;
                
                _ui.configProfilesDropdown->clear();
                for (const auto& profile : configs) {
                    _ui.configProfilesDropdown->addItem(QString::fromStdString(profile));
                }
                
                _isUpdatingDropdown = false;
                RCLCPP_INFO(_node->get_logger(), "Successfully refreshed configuration dropdown profiles.");
            }
        });
}

void QParamSaverWidget::onProfileSwitch(const QString &text)
{
    if (_isUpdatingDropdown || text.isEmpty()) {
        return;
    }

    if (!_setParamClient->wait_for_service(std::chrono::milliseconds(500)))
    {
        RCLCPP_ERROR(_node->get_logger(), "Unable to change profile: SetParameter service offline!");
        return;
    }

    this->setUiEnabled(false);

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter param;
    param.name = "config_name";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    param.value.string_value = text.toStdString();
    request->parameters.push_back(param);

    std::string profile_name = text.toStdString();
    _setParamClient->async_send_request(request,
        [this, profile_name](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
            this->setUiEnabled(true);

            auto response = future.get();
            if (!response->results.empty() && response->results[0].successful) {
                RCLCPP_INFO(_node->get_logger(), "Profile successfully changed to '%s'", profile_name.c_str());
                
                QTimer::singleShot(200, this, &QParamSaverWidget::reloadProfiles);
            } else {
                std::string reason = response->results.empty() ? "Unknown Error" : response->results[0].reason;
                RCLCPP_ERROR(_node->get_logger(), "Failed to switch profile: %s", reason.c_str());
            }
        });
}

void QParamSaverWidget::setUiEnabled(bool enabled)
{
    _ui.configProfilesDropdown->setEnabled(enabled);
    _ui.refreshButton->setEnabled(enabled);
    _ui.globalSaveButton->setEnabled(enabled);
}