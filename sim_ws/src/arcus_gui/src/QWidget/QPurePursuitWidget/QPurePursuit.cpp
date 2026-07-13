#include "QPurePursuit.hpp"
#include <yaml-cpp/yaml.h>

QPurePursuitWidget::QPurePursuitWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _maxLookaheadSlider(parent_, node_, MAX_LOOKAHEAD_PARAM_NAME, PURE_PURSUIT_NODE_NAME, MAX_LOOKAHEAD_MIN, MAX_LOOKAHEAD_MAX, MAX_LOOKAHEAD_DEFAULT, MAX_LOOKAHEAD_PRECISION),
    _speedMaxSlider(parent_, node_, MAX_SPEED_PARAM_NAME, PURE_PURSUIT_NODE_NAME, MAX_SPEED_MIN, MAX_SPEED_MAX, MAX_SPEED_DEFAULT, MAX_SPEED_PRECISION),
    _speedLookeaheadGainSlider(parent_, node_, SPEED_LOOKAHEAD_GAIN_PARAM_NAME, PURE_PURSUIT_NODE_NAME, SPEED_LOOKAHEAD_GAIN_MIN, SPEED_LOOKAHEAD_GAIN_MAX, SPEED_LOOKAHEAD_GAIN_DEFAULT, SPEED_LOOKAHEAD_GAIN_PRECISION),
    _accelLatSlider(parent_, node_, ACCEL_LAT_PARAM_NAME, PURE_PURSUIT_NODE_NAME, ACCEL_LAT_MIN, ACCEL_LAT_MAX, ACCEL_LAT_DEFAULT, ACCEL_LAT_PRECISION),
    _accelLongSlider(parent_, node_, ACCEL_LONG_PARAM_NAME, PURE_PURSUIT_NODE_NAME, ACCEL_LONG_MIN, ACCEL_LONG_MAX, ACCEL_LONG_DEFAULT, ACCEL_LONG_PRECISION),
    _brakeLongSlider(parent_, node_, BRAKE_LONG_PARAM_NAME, PURE_PURSUIT_NODE_NAME, BRAKE_LONG_MIN, BRAKE_LONG_MAX, BRAKE_LONG_DEFAULT, BRAKE_LONG_PRECISION)
{
    _ui.setupUi(this);

    _ui.sliderLayout->addWidget(&_maxLookaheadSlider);
    _ui.sliderLayout->addWidget(&_speedMaxSlider);
    _ui.sliderLayout->addWidget(&_speedLookeaheadGainSlider);
    _ui.sliderLayout->addWidget(&_accelLatSlider);
    _ui.sliderLayout->addWidget(&_accelLongSlider);
    _ui.sliderLayout->addWidget(&_brakeLongSlider);

    this->connectSignals();
}

void QPurePursuitWidget::connectSignals(void)
{
    connect(_ui.applyAllPB, &QPushButton::clicked, this, &QPurePursuitWidget::onApplyAllClicked);
}

void QPurePursuitWidget::onApplyAllClicked(void)
{
    const auto sliders = this->findChildren<QParamSlider*>();
    for (auto* slider : sliders)
    {
        slider->onApplyClicked();
    }
}

void QPurePursuitWidget::refreshSliderValues(void)
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