#include "QGapFollow.hpp"
#include <yaml-cpp/yaml.h>

QGapFollowWidget::QGapFollowWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _maxSpeedSlider(parent_, node_, MAX_SPEED_PARAM_NAME, GAP_FOLLOW_NODE_NAME, MAX_SPEED_MIN, MAX_SPEED_MAX, MAX_SPEED_DEFAULT, SPEED_PRECISION),
    _distanceSpeedGainSlider(parent_, node_, DISTANCE_SPEED_GAIN_PARAM_NAME, GAP_FOLLOW_NODE_NAME, DISTANCE_SPEED_GAIN_MIN, DISTANCE_SPEED_GAIN_MAX, DISTANCE_SPEED_GAIN_DEFAULT, DISTANCE_SPEED_GAIN_PRECISION),
    _bubbleRadiusSlider(parent_, node_, BUBBLE_RADIUS_PARAM_NAME, GAP_FOLLOW_NODE_NAME, BUBBLE_RADIUS_MIN, BUBBLE_RADIUS_MAX, BUBBLE_RADIUS_DEFAULT, BUBBLE_RADIUS_PRECISION),
    _staticFrictionCoeffSlider(parent_, node_, STATIC_FRICTION_COEFF, GAP_FOLLOW_NODE_NAME, STATIC_FRICTION_COEFF_MIN, STATIC_FRICTION_COEFF_MAX, STATIC_FRICTION_COEFF_DEFAULT, STATIC_FRICTION_COEFF_PRECISION)
{
    _ui.setupUi(this);

    _ui.sliderLayout->addWidget(&_maxSpeedSlider);
    _ui.sliderLayout->addWidget(&_distanceSpeedGainSlider);
    _ui.sliderLayout->addWidget(&_bubbleRadiusSlider);
    _ui.sliderLayout->addWidget(&_staticFrictionCoeffSlider);

    this->connectSignals();
}

void QGapFollowWidget::connectSignals(void)
{
    connect(_ui.applyAllPB, &QPushButton::clicked, this, &QGapFollowWidget::onApplyAllClicked);
}

void QGapFollowWidget::onApplyAllClicked(void)
{
    const auto sliders = this->findChildren<QParamSlider*>();
    for (auto* slider : sliders)
    {
        slider->onApplyClicked();
    }
}

void QGapFollowWidget::refreshSliderValues(void)
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