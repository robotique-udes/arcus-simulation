#include "QPurePursuit.hpp"

QPurePursuitWidget::QPurePursuitWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _lookaheadSlider(parent_, node_, LOOKAHEAD_PARAM_NAME, PURE_PURSUIT_NODE_NAME, LOOKAHEAD_MIN, LOOKAHEAD_MAX, LOOKAHEAD_DEFAULT, LOOKAHEAD_PRECISION),
    _maxLookaheadSlider(parent_, node_, MAX_LOOKAHEAD_PARAM_NAME, PURE_PURSUIT_NODE_NAME, MAX_LOOKAHEAD_MIN, MAX_LOOKAHEAD_MAX, MAX_LOOKAHEAD_DEFAULT, MAX_LOOKAHEAD_PRECISION),
    _minLookaheadSlider(parent_, node_, MIN_LOOKAHEAD_PARAM_NAME, PURE_PURSUIT_NODE_NAME, MIN_LOOKAHEAD_MIN, MIN_LOOKAHEAD_MAX, MIN_LOOKAHEAD_DEFAULT, MIN_LOOKAHEAD_PRECISION),
    _speedLookeaheadGainSlider(parent_, node_, SPEED_LOOKAHEAD_GAIN_PARAM_NAME, PURE_PURSUIT_NODE_NAME, SPEED_LOOKAHEAD_GAIN_MIN, SPEED_LOOKAHEAD_GAIN_MAX, SPEED_LOOKAHEAD_GAIN_DEFAULT, SPEED_LOOKAHEAD_GAIN_PRECISION),
    _accelLatSlider(parent_, node_, ACCEL_LAT_PARAM_NAME, PURE_PURSUIT_NODE_NAME, ACCEL_LAT_MIN, ACCEL_LAT_MAX, ACCEL_LAT_DEFAULT, ACCEL_LAT_PRECISION),
    _accelLongSlider(parent_, node_, ACCEL_LONG_PARAM_NAME, PURE_PURSUIT_NODE_NAME, ACCEL_LONG_MIN, ACCEL_LONG_MAX, ACCEL_LONG_DEFAULT, ACCEL_LONG_PRECISION)
{
    _ui.setupUi(this);
    _ui.sliderLayout->addWidget(&_lookaheadSlider);
    _ui.sliderLayout->addWidget(&_maxLookaheadSlider);
    _ui.sliderLayout->addWidget(&_minLookaheadSlider);
    _ui.sliderLayout->addWidget(&_speedLookeaheadGainSlider);
    _ui.sliderLayout->addWidget(&_accelLatSlider);
    _ui.sliderLayout->addWidget(&_accelLongSlider);
}