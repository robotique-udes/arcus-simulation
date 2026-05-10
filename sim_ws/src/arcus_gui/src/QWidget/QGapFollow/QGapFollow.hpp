#ifndef Q_GAP_FOLLOW_WIDGET_HPP
#define Q_GAP_FOLLOW_WIDGET_HPP

#include "UI_GapFollowWidget.h"
#include "Global/Helper/QParamSlider/QParamSlider.hpp"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QGapFollowWidget : public QWidget
{
    Q_OBJECT

    static constexpr const char* GAP_FOLLOW_NODE_NAME = "/arcus/gap_follow";
    static constexpr const char* MAX_SPEED_PARAM_NAME = "max_speed";
    static constexpr const char* DISTANCE_SPEED_GAIN_PARAM_NAME = "speed_distance_factor";
    static constexpr const char* BUBBLE_RADIUS_PARAM_NAME = "bubble_radius";
    static constexpr const char* STATIC_FRICTION_COEFF = "static_friction_coeff";

    static constexpr float MAX_SPEED_DEFAULT = 5;
    static constexpr float DISTANCE_SPEED_GAIN_DEFAULT = 1.2;
    static constexpr float BUBBLE_RADIUS_DEFAULT = 0.3;
    static constexpr float STATIC_FRICTION_COEFF_DEFAULT = 0.7;

    static constexpr float MAX_SPEED_MIN = 0.0;
    static constexpr float DISTANCE_SPEED_GAIN_MIN = 0.0;
    static constexpr float BUBBLE_RADIUS_MIN = 0.0;
    static constexpr float STATIC_FRICTION_COEFF_MIN = 0.0;

    static constexpr float MAX_SPEED_MAX = 20.0;
    static constexpr float DISTANCE_SPEED_GAIN_MAX = 10.0;
    static constexpr float BUBBLE_RADIUS_MAX = 1.0;
    static constexpr float STATIC_FRICTION_COEFF_MAX = 5.0;

    static constexpr float SPEED_PRECISION = 0.05;
    static constexpr float DISTANCE_SPEED_GAIN_PRECISION = 0.05;
    static constexpr float BUBBLE_RADIUS_PRECISION = 0.05;
    static constexpr float STATIC_FRICTION_COEFF_PRECISION = 0.05;


  public:
    QGapFollowWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  private:
    void setupUI(void);

    std::shared_ptr<rclcpp::Node> _node;

    QParamSlider _maxSpeedSlider;
    QParamSlider _distanceSpeedGainSlider;
    QParamSlider _bubbleRadiusSlider;
    QParamSlider _staticFrictionCoeffSlider;

    Ui::gapFollowWidget _ui;
};

#endif  // Q_GAP_FOLLOW_WIDGET_HPP
