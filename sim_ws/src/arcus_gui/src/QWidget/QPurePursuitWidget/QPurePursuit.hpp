#ifndef Q_PURE_PURSUIT_WIDGET_HPP
#define Q_PURE_PURSUIT_WIDGET_HPP

#include "UI_PurePursuitWidget.h"
#include "Global/Helper/QParamSlider/QParamSlider.hpp"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QPurePursuitWidget : public QWidget
{
    Q_OBJECT

  public:
    QPurePursuitWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  private:
    void setupUI(void);

    std::shared_ptr<rclcpp::Node> _node;

    QParamSlider _lookaheadSlider;
    QParamSlider _maxLookaheadSlider;
    QParamSlider _minLookaheadSlider;
    QParamSlider _speedLookeaheadGainSlider;
    QParamSlider _accelLatSlider;
    QParamSlider _accelLongSlider;



    Ui::purePursuitWidget _ui;
};

#endif  // Q_PURE_PURSUIT_WIDGET_HPP
