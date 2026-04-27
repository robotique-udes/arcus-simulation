#ifndef Q_PURE_PURSUIT_WIDGET_HPP
#define Q_PURE_PURSUIT_WIDGET_HPP

#include "UI_PurePursuitWidget.h"
#include "Global/Helper/QParamSlider/QParamSlider.hpp"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QPurePursuitWidget : public QWidget
{
    Q_OBJECT

    static constexpr const char* PURE_PURSUIT_NODE_NAME = "/arcus/pure_pursuit";

    static constexpr const char* MAX_LOOKAHEAD_PARAM_NAME = "max_lookahead_distance_m";
    static constexpr const char* MIN_LOOKAHEAD_PARAM_NAME = "min_lookahead_distance_m";
    static constexpr const char* SPEED_LOOKAHEAD_GAIN_PARAM_NAME = "lookahead_distance_gain";
    static constexpr const char* ACCEL_LAT_PARAM_NAME = "a_lat_max";
    static constexpr const char* ACCEL_LONG_PARAM_NAME = "a_accel_max";
    static constexpr const char* BRAKE_LONG_PARAM_NAME = "a_brake_max";

    static constexpr float MAX_LOOKAHEAD_DEFAULT = 5.0;
    static constexpr float MIN_LOOKAHEAD_DEFAULT = 0.5;
    static constexpr float SPEED_LOOKAHEAD_GAIN_DEFAULT = 0.5;
    static constexpr float ACCEL_LAT_DEFAULT = 2.0;
    static constexpr float ACCEL_LONG_DEFAULT = 2.0;
    static constexpr float BRAKE_LONG_DEFAULT = 2.0;

    static constexpr float MAX_LOOKAHEAD_MIN = 0.0;
    static constexpr float MIN_LOOKAHEAD_MIN = 0.0;
    static constexpr float SPEED_LOOKAHEAD_GAIN_MIN = 0.0;
    static constexpr float ACCEL_LAT_MIN = 0.0;
    static constexpr float ACCEL_LONG_MIN = 0.0;
    static constexpr float BRAKE_LONG_MIN = 0.0;


    static constexpr float MAX_LOOKAHEAD_MAX = 10.0;
    static constexpr float MIN_LOOKAHEAD_MAX = 10.0;
    static constexpr float SPEED_LOOKAHEAD_GAIN_MAX = 20.0;
    static constexpr float ACCEL_LAT_MAX = 20.0;
    static constexpr float ACCEL_LONG_MAX = 20.0;
    static constexpr float BRAKE_LONG_MAX = 20.0;

    static constexpr float MAX_LOOKAHEAD_PRECISION = 0.1;
    static constexpr float MIN_LOOKAHEAD_PRECISION = 0.1;
    static constexpr float SPEED_LOOKAHEAD_GAIN_PRECISION = 0.1;
    static constexpr float ACCEL_LAT_PRECISION = 0.1;
    static constexpr float ACCEL_LONG_PRECISION = 0.1;
    static constexpr float BRAKE_LONG_PRECISION = 0.1;

  public:
    QPurePursuitWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  private:
    void setupUI(void);

    std::shared_ptr<rclcpp::Node> _node;

    QParamSlider _maxLookaheadSlider;
    QParamSlider _minLookaheadSlider;
    QParamSlider _speedLookeaheadGainSlider;
    QParamSlider _accelLatSlider;
    QParamSlider _accelLongSlider;
    QParamSlider _brakeLongSlider;

    Ui::purePursuitWidget _ui;
};

#endif  // Q_PURE_PURSUIT_WIDGET_HPP
