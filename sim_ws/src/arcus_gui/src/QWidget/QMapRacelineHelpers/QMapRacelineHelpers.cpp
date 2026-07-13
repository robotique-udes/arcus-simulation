#include "QMapRacelineHelpers.hpp"

QMapRacelineHelpers::QMapRacelineHelpers(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _racelineHelper(this, "Raceline Helpers", RACELINE_HELPER_CMD, true),
    _ttcDecayRateSlider(parent_, node_, TTC_DECAY_RATE_PARAM_NAME, PURE_PURSUIT_NODE_NAME, TTC_DECAY_RATE_MIN, TTC_DECAY_RATE_MAX, TTC_DECAY_RATE_DEFAULT, TTC_DECAY_RATE_PRECISION),
    _maxRiskSlider(parent_, node_, MAX_RISK_PARAM_NAME, MASTER_NODE_NAME, MAX_RISK_MIN, MAX_RISK_MAX, MAX_RISK_DEFAULT, MAX_RISK_PRECISION)
{
    _ui.setupUi(this);
    _ui.buttonsLayout->addWidget(&_racelineHelper);
    _ui.buttonsLayout->addWidget(&_ttcDecayRateSlider);
    _ui.buttonsLayout->addWidget(&_maxRiskSlider);

    this->connectSignals();
}

void QMapRacelineHelpers::connectSignals(void)
{
    connect(_ui.applyAllPB, &QPushButton::clicked, this, &QMapRacelineHelpers::onApplyAllClicked);
}

void QMapRacelineHelpers::onApplyAllClicked(void)
{
    const auto sliders = this->findChildren<QParamSlider*>();
    for (auto* slider : sliders)
    {
        slider->onApplyClicked();
    }
}

void QMapRacelineHelpers::refreshSliderValues(void)
{
    const auto sliders = this->findChildren<QParamSlider*>();
    if (sliders.empty()) return;
    
    std::map<std::string, std::vector<QParamSlider*>> node_to_sliders_map;
    for (auto* slider : sliders) {
        node_to_sliders_map[slider->getRemoteNodeName()].push_back(slider);
    }
    
    for (const auto& [remote_node, node_sliders] : node_to_sliders_map) 
    {
        std::string service_name = remote_node + "/get_parameters";

        if (_param_clients.find(service_name) == _param_clients.end()) {
            _param_clients[service_name] = _node->create_client<rcl_interfaces::srv::GetParameters>(service_name);
        }
        auto client = _param_clients[service_name];
        
        if (!client->wait_for_service(std::chrono::milliseconds(300))) {
            RCLCPP_WARN(_node->get_logger(), "Dynamic refresh skipped: %s is offline.", remote_node.c_str());
            continue;
        }
        
        auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        for (auto* slider : node_sliders) {
            request->names.push_back(slider->getParamName());
        }
        
        client->async_send_request(request,
            [this, node_sliders](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture future) {
                auto response = future.get();
                
                if (response->values.size() != node_sliders.size()) return;
                
                for (size_t i = 0; i < node_sliders.size(); ++i) {
                    float val = 0.0f;
                    const auto& p_val = response->values[i];
                    
                    if (p_val.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
                        val = static_cast<float>(p_val.double_value);
                    } else if (p_val.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
                        val = static_cast<float>(p_val.integer_value);
                    } else {
                        continue;
                    }
                    
                    QParamSlider* target_slider = node_sliders[i];

                    QMetaObject::invokeMethod(this, [target_slider, val]() {
                        target_slider->updateValue(val);
                    }, Qt::QueuedConnection);
                }
            });
    }
}