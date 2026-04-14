#ifndef Q_PARAM_SLIDER_HPP
#define Q_PARAM_SLIDER_HPP

#include "UI_ParamSlider.h"

#include <rclcpp/rclcpp.hpp>
#include <QtWidgets/QWidget>

class QParamSlider : public QWidget
{
    Q_OBJECT

  public:
    QParamSlider(QWidget* parent_, std::shared_ptr<rclcpp::Node> node_);


  private:
    void setupUi(void);

    std::shared_ptr<rclcpp::Node> _node;
    Ui::paramSlider _ui;
};

#endif  // Q_PARAM_SLIDER_HPP
