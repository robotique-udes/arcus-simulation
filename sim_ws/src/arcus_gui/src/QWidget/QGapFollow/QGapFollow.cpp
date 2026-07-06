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