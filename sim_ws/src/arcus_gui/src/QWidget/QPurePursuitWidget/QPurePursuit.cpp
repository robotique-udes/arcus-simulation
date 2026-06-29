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
}

void QPurePursuitWidget::connectSignals(void)
{
    connect(&_applyAllPB, &QPushButton::clicked, this, &QGapFollowWidget::onApplyAllClicked);
}

void QPurePursuitWidget::onApplyAllClicked(void)
{
    const auto sliders = this->findChildren<QParamSlider*>();
    for (auto* slider : sliders)
    {
        slider->onApplyClicked();
    }
}