#ifndef Q_PARAM_SAVER_WIDGET_HPP
#define Q_PARAM_SAVER_WIDGET_HPP

#include "UI_ParamSaverWidget.h"
#include "std_srvs/srv/trigger.hpp"

#include <QtWidgets/QWidget>
#include <QInputDialog>
#include <QMessageBox>
#include <QMetaObject>
#include <rclcpp/rclcpp.hpp>

class QParamSaverWidget : public QWidget
{
    Q_OBJECT

  public:
    QParamSaverWidget(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_);
  
  signals:
    void profileReloaded();
  private:
    void connectSignals(void);
    void onGlobalSaveClicked(void);
    Q_INVOKABLE void reloadProfiles(void);
    void onProfileSwitch(const QString &text);
    void onAddProfileClicked(void);
    void setUiEnabled(bool enabled);

    std::shared_ptr<rclcpp::Node> _node;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _paramSaverClient;
    rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr _getParamClient;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr _setParamClient;

    bool _isUpdatingDropdown = false;

    Ui::paramSaverWidget _ui;
};

#endif  // Q_PARAM_SAVER_WIDGET_HPP
