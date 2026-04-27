#include "QPurePursuit.hpp"

QPurePursuitWidget::QPurePursuitWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _lookaheadSlider(parent_, node_),
    _maxLookaheadSlider(parent_, node_),
    _minLookaheadSlider(parent_, node_),
    _speedLookeaheadGainSlider(parent_, node_),
    _accelLatSlider(parent_, node_),
    _accelLongSlider(parent_, node_)
{
    _ui.setupUi(this);
    _ui.sliderLayout->addWidget(&_lookaheadSlider);
    _ui.sliderLayout->addWidget(&_maxLookaheadSlider);
    _ui.sliderLayout->addWidget(&_minLookaheadSlider);
    _ui.sliderLayout->addWidget(&_speedLookeaheadGainSlider);
    _ui.sliderLayout->addWidget(&_accelLatSlider);
    _ui.sliderLayout->addWidget(&_accelLongSlider);
}