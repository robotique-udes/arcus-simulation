#ifndef Q_PARAM_SAVER_WIDGET_HPP
#define Q_PARAM_SAVER_WIDGET_HPP

#include "UI_ParamSaverWidget.h"
#include "std_srvs/srv/trigger.hpp"

#include <QtWidgets/QWidget>
#include <rclcpp/rclcpp.hpp>

class QParamSaverWidget : public QWidget
{
    Q_OBJECT

  public:
    QParamSaverWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);

  private:
    void connectSignals(void);
    void onGlobalSaveClicked(void);

    std::shared_ptr<rclcpp::Node> _node;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _paramSaverClient;

    Ui::paramSaverWidget _ui;
};

#endif  // Q_PARAM_SAVER_WIDGET_HPP
